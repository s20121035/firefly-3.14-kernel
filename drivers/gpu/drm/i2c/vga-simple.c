/*
 * Simple vga encoder driver
 *
 * Copyright (C) 2014 Heiko Stuebner <heiko@sntech.de>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/component.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>

#define connector_to_vga_simple(x) container_of(x, struct vga_simple, connector)
#define encoder_to_vga_simple(x) container_of(x, struct vga_simple, encoder)

struct vga_simple {
	struct drm_connector connector;
	struct drm_encoder encoder;

	struct device *dev;
	struct i2c_adapter *ddc;

	struct regulator *vaa_reg;
	struct gpio_desc *enable_gpio;

	struct mutex enable_lock;
	bool enabled;
};

/*
 * Connector functions
 */

enum drm_connector_status vga_simple_detect(struct drm_connector *connector,
					    bool force)
{
	struct vga_simple *vga = connector_to_vga_simple(connector);

	if (!vga->ddc)
		return connector_status_unknown;

	if (drm_probe_ddc(vga->ddc))
		return connector_status_connected;

	return connector_status_disconnected;
}

void vga_simple_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

struct drm_connector_funcs vga_simple_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = vga_simple_detect,
	.destroy = vga_simple_connector_destroy,
};

/*
 * Connector helper functions
 */

static int vga_simple_connector_get_modes(struct drm_connector *connector)
{
	struct vga_simple *vga = connector_to_vga_simple(connector);
	struct edid *edid;
	int ret = 0;

	if (!vga->ddc)
		return 0;

	edid = drm_get_edid(connector, vga->ddc);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	return ret;
}

static int vga_simple_connector_mode_valid(struct drm_connector *connector,
					struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_encoder
*vga_simple_connector_best_encoder(struct drm_connector *connector)
{
	struct vga_simple *vga = connector_to_vga_simple(connector);

	return &vga->encoder;
}

static struct drm_connector_helper_funcs vga_simple_connector_helper_funcs = {
	.get_modes = vga_simple_connector_get_modes,
	.best_encoder = vga_simple_connector_best_encoder,
	.mode_valid = vga_simple_connector_mode_valid,
};

/*
 * Encoder functions
 */

static void vga_simple_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs vga_simple_encoder_funcs = {
	.destroy = vga_simple_encoder_destroy,
};

/*
 * Encoder helper functions
 */

static void vga_simple_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct vga_simple *vga = encoder_to_vga_simple(encoder);

	mutex_lock(&vga->enable_lock);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		if (vga->enabled)
			goto out;

		if (!IS_ERR(vga->vaa_reg))
			regulator_enable(vga->vaa_reg);

		if (vga->enable_gpio)
			gpiod_set_value(vga->enable_gpio, 1);

		vga->enabled = true;
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		if (!vga->enabled)
			goto out;

		if (vga->enable_gpio)
			gpiod_set_value(vga->enable_gpio, 0);

		if (!IS_ERR(vga->vaa_reg))
			regulator_enable(vga->vaa_reg);

		vga->enabled = false;
		break;
	default:
		break;
	}

out:
	mutex_unlock(&vga->enable_lock);
}

static bool vga_simple_mode_fixup(struct drm_encoder *encoder,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void vga_simple_encoder_prepare(struct drm_encoder *encoder)
{
}

static void vga_simple_encoder_mode_set(struct drm_encoder *encoder,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode)
{
}

static void vga_simple_encoder_commit(struct drm_encoder *encoder)
{
	vga_simple_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void vga_simple_encoder_disable(struct drm_encoder *encoder)
{
	vga_simple_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static const struct drm_encoder_helper_funcs vga_simple_encoder_helper_funcs = {
	.dpms = vga_simple_encoder_dpms,
	.mode_fixup = vga_simple_mode_fixup,
	.prepare = vga_simple_encoder_prepare,
	.mode_set = vga_simple_encoder_mode_set,
	.commit = vga_simple_encoder_commit,
	.disable = vga_simple_encoder_disable,
};

/*
 * Component helper functions
 */

static int vga_simple_bind(struct device *dev, struct device *master,
				 void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = data;

	struct device_node *ddc_node, *np = pdev->dev.of_node;
	struct vga_simple *vga;
	int ret;

	if (!np)
		return -ENODEV;

	vga = devm_kzalloc(dev, sizeof(*vga), GFP_KERNEL);
	if (!vga)
		return -ENOMEM;

	vga->dev = dev;
	dev_set_drvdata(dev, vga);
	mutex_init(&vga->enable_lock);

	vga->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						   GPIOD_OUT_LOW);
	if (IS_ERR(vga->enable_gpio)) {
		ret = PTR_ERR(vga->enable_gpio);
		dev_err(dev, "failed to request GPIO: %d\n", ret);
		return ret;
	}

	vga->enabled = false;

	ddc_node = of_parse_phandle(np, "ddc-i2c-bus", 0);
	if (ddc_node) {
		vga->ddc = of_find_i2c_adapter_by_node(ddc_node);
		of_node_put(ddc_node);
		if (!vga->ddc) {
			dev_dbg(vga->dev, "failed to read ddc node\n");
			return -EPROBE_DEFER;
		}
	} else {
		dev_dbg(vga->dev, "no ddc property found\n");
	}

	vga->vaa_reg = devm_regulator_get_optional(dev, "vaa");

	vga->encoder.possible_crtcs = drm_of_find_possible_crtcs(drm, np);
	vga->encoder.of_node = np;
	vga->connector.polled = DRM_CONNECTOR_POLL_CONNECT |
				DRM_CONNECTOR_POLL_DISCONNECT;

	drm_encoder_helper_add(&vga->encoder, &vga_simple_encoder_helper_funcs);
	drm_encoder_init(drm, &vga->encoder, &vga_simple_encoder_funcs,
			 DRM_MODE_ENCODER_DAC);

	drm_connector_helper_add(&vga->connector,
				 &vga_simple_connector_helper_funcs);
	drm_connector_init(drm, &vga->connector, &vga_simple_connector_funcs,
			   DRM_MODE_CONNECTOR_VGA);

	drm_mode_connector_attach_encoder(&vga->connector, &vga->encoder);

	return 0;
}

static void vga_simple_unbind(struct device *dev, struct device *master,
				    void *data)
{
	struct vga_simple *vga = dev_get_drvdata(dev);

	vga->connector.funcs->destroy(&vga->connector);
	vga->encoder.funcs->destroy(&vga->encoder);
}

static const struct component_ops vga_simple_ops = {
	.bind = vga_simple_bind,
	.unbind = vga_simple_unbind,
};

static int vga_simple_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &vga_simple_ops);
}

static int vga_simple_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &vga_simple_ops);

	return 0;
}

static const struct of_device_id vga_simple_ids[] = {
	{ .compatible = "adi,adv7123", },
	{},
};
MODULE_DEVICE_TABLE(of, vga_simple_ids);

static struct platform_driver vga_simple_driver = {
	.probe  = vga_simple_probe,
	.remove = vga_simple_remove,
	.driver = {
		.name = "vga-simple",
		.of_match_table = vga_simple_ids,
	},
};
module_platform_driver(vga_simple_driver);

MODULE_AUTHOR("Heiko Stuebner <heiko@sntech.de>");
MODULE_DESCRIPTION("Simple vga converter");
MODULE_LICENSE("GPL");
