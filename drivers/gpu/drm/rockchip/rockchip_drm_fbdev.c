/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
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

#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
//#include <linux/ump.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_fb.h"

#define PREFERRED_BPP		32
#define to_drm_private(x) \
		container_of(x, struct rockchip_drm_private, fbdev_helper)

//struct ump_dd_handle ump_wrapped_buffer;
//struct ump_dd_physical_block ump_memory_description;

static int rockchip_fbdev_mmap(struct fb_info *info,
			       struct vm_area_struct *vma)
{
	struct drm_fb_helper *helper = info->par;
	struct rockchip_drm_private *private = to_drm_private(helper);

	return rockchip_gem_mmap_buf(private->fbdev_bo, vma);
}

#ifdef CONFIG_UMP
static int drm_fb_ioctl(struct fb_info *info, unsigned int cmd,
		       unsigned long arg)
{
	//struct fb_fix_screeninfo *fix = &info->fix;
    switch (cmd) 
    {
        case GET_UMP_SECURE_ID:
        case GET_UMP_SECURE_ID_1:
        case GET_UMP_SECURE_ID_2:
        case GET_UMP_SECURE_ID_3:
            printk("drm fb ioctl %d \n",cmd);
            struct drm_fb_helper *helper = info->par;
            struct rockchip_drm_private *private = to_drm_private(helper);
            u32 __user *psecureid = (u32 __user *) arg;
            ump_secure_id secure_id;

            secure_id = ump_dd_secure_id_get( private->ump_wrapped_buffer );
            put_user( (unsigned int)secure_id, psecureid );
            break;
        default:
            break;
    }
}
#endif


static struct fb_ops rockchip_drm_fbdev_ops = {
	.owner		= THIS_MODULE,
	.fb_mmap	= rockchip_fbdev_mmap,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
#ifdef CONFIG_UMP
    .fb_ioctl       = drm_fb_ioctl,
    .fb_compat_ioctl = drm_fb_ioctl,
#endif
	.fb_check_var	= drm_fb_helper_check_var,
	.fb_set_par	= drm_fb_helper_set_par,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= drm_fb_helper_pan_display,
	.fb_setcmap	= drm_fb_helper_setcmap,
};



static int rockchip_drm_fbdev_create(struct drm_fb_helper *helper,
				     struct drm_fb_helper_surface_size *sizes)
{
	struct rockchip_drm_private *private = to_drm_private(helper);
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct drm_device *dev = helper->dev;
	struct rockchip_gem_object *rk_obj;
	struct drm_framebuffer *fb;
	unsigned int bytes_per_pixel;
	unsigned long offset;
	struct fb_info *fbi;
	size_t size;
	int ret;

	bytes_per_pixel = DIV_ROUND_UP(sizes->surface_bpp, 8);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = sizes->surface_width * bytes_per_pixel;
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
		sizes->surface_depth);

#ifdef CONFIG_UMP
	size = mode_cmd.pitches[0] * mode_cmd.height * 2;
#else
	size = mode_cmd.pitches[0] * mode_cmd.height;
#endif

	rk_obj = rockchip_gem_create_object(dev, size, true);
	if (IS_ERR(rk_obj))
		return -ENOMEM;

	private->fbdev_bo = &rk_obj->base;

	fbi = framebuffer_alloc(0, dev->dev);
	if (!fbi) {
		dev_err(dev->dev, "Failed to allocate framebuffer info.\n");
		ret = -ENOMEM;
		goto err_rockchip_gem_free_object;
	}

	helper->fb = rockchip_drm_framebuffer_init(dev, &mode_cmd,
						   private->fbdev_bo);
	if (IS_ERR(helper->fb)) {
		dev_err(dev->dev, "Failed to allocate DRM framebuffer.\n");
		ret = PTR_ERR(helper->fb);
		goto err_framebuffer_release;
	}

	helper->fbdev = fbi;

	fbi->par = helper;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->fbops = &rockchip_drm_fbdev_ops;

	ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (ret) {
		dev_err(dev->dev, "Failed to allocate color map.\n");
		goto err_drm_framebuffer_unref;
	}

	fb = helper->fb;
	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, fb->width, fb->height);

	offset = fbi->var.xoffset * bytes_per_pixel;
	offset += fbi->var.yoffset * fb->pitches[0];

	dev->mode_config.fb_base = 0;
	fbi->screen_base = rk_obj->kvaddr + offset;
	fbi->screen_size = rk_obj->base.size;
	fbi->fix.smem_len = rk_obj->base.size;

#ifdef CONFIG_UMP
    private->ump_memory_description.addr = fbi->screen_base;
    private->ump_memory_description.size = fbi->screen_size;
    private->ump_wrapped_buffer = ump_dd_handle_create_from_phys_blocks(
                      &private->ump_memory_description, 1);
#endif


	DRM_DEBUG_KMS("FB [%dx%d]-%d kvaddr=%p offset=%ld size=%d\n",
		      fb->width, fb->height, fb->depth, rk_obj->kvaddr,
		      offset, size);

	fbi->skip_vt_switch = true;
	fb_prepare_logo(fbi, 0);
	fb_show_logo(fbi, 0);

	return 0;

err_drm_framebuffer_unref:
	drm_framebuffer_unreference(helper->fb);
err_framebuffer_release:
	framebuffer_release(fbi);
err_rockchip_gem_free_object:
	rockchip_gem_free_object(&rk_obj->base);
	return ret;
}

static const struct drm_fb_helper_funcs rockchip_drm_fb_helper_funcs = {
	.fb_probe = rockchip_drm_fbdev_create,
};

int rockchip_drm_fbdev_init(struct drm_device *dev)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct drm_fb_helper *helper;
	unsigned int num_crtc;
	int ret;

	if (!dev->mode_config.num_crtc || !dev->mode_config.num_connector)
		return -EINVAL;

	num_crtc = dev->mode_config.num_crtc;

	helper = &private->fbdev_helper;

	drm_fb_helper_prepare(dev, helper, &rockchip_drm_fb_helper_funcs);

	ret = drm_fb_helper_init(dev, helper, num_crtc, ROCKCHIP_MAX_CONNECTOR);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to initialize drm fb helper - %d.\n",
			ret);
		return ret;
	}

	ret = drm_fb_helper_single_add_all_connectors(helper);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to add connectors - %d.\n", ret);
		goto err_drm_fb_helper_fini;
	}

	/* disable all the possible outputs/crtcs before entering KMS mode */
	drm_helper_disable_unused_functions(dev);

	ret = drm_fb_helper_initial_config(helper, PREFERRED_BPP);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to set initial hw config - %d.\n",
			ret);
		goto err_drm_fb_helper_fini;
	}

	return 0;

err_drm_fb_helper_fini:
	drm_fb_helper_fini(helper);
	return ret;
}

void rockchip_drm_fbdev_fini(struct drm_device *dev)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct drm_fb_helper *helper;

	helper = &private->fbdev_helper;

	if (helper->fbdev) {
		struct fb_info *info;
		int ret;

		info = helper->fbdev;
		ret = unregister_framebuffer(info);
		if (ret < 0)
			DRM_DEBUG_KMS("failed unregister_framebuffer() - %d\n",
				      ret);

		if (info->cmap.len)
			fb_dealloc_cmap(&info->cmap);

		framebuffer_release(info);
	}

	if (helper->fb)
		drm_framebuffer_unreference(helper->fb);

	drm_fb_helper_fini(helper);
}
