#ifndef __OMAP_PANEL_DSI_H__
#define __OMAP_PANEL_DSI_H__

#define DSI_FPS_DATA(_name, _id, _regm, _tlpx, _tclk_zero, _tclk_prepare, _tclk_trail, _ths_zero, _ths_prepare, _ths_trail, _ths_exit) \
 struct panel_dsi_fps_data dsi_fps_data_##_name = { .name = #_id, .regm = _regm, .tlpx = _tlpx, \
	.tclk = { .zero = _tclk_zero, .prepare = _tclk_prepare, .trail = _tclk_trail }, \
	.ths = { .zero = _ths_zero, .prepare = _ths_prepare, .trail = _ths_trail, .exit = _ths_exit } }

struct panel_dsi_fps_data {
	const char *name; /* Name of fps setting - in frames per second */
	u16 regm;

	u8 tlpx;
	struct {
		u8 zero;
		u8 prepare;
		u8 trail;
	} tclk;
	struct {
		u8 zero;
		u8 prepare;
		u8 trail;
		u8 exit;
	} ths;
};

struct panel_dsi_data {
	u16 x_res; /* screen width in pixels */
	u16 y_res; /* screen height in pixels */

	u32 pixel_clock; /* pixel clock in KHz */

	u16 hfp; /* horizontal front porch in clocks */
	u16 hsw; /* horizontal sync width in clocks */
	u16 hbp; /* horizontal back porch in clocks */

	u16 vfp; /* vertical front porch in lines */
	u16 vsw; /* vertical sync width in lines */
	u16 vbp; /* vertical back porch in lines */

	u32 width_in_um;  /* physical dimensions - width in micrometers */
	u32 height_in_um; /* physical dimensions - height in micrometers */
};

#endif /* __OMAP_PANEL_DSI_H__ */

