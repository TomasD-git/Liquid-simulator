#pragma once
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI _bus;
  
public:
  LGFX(void) {
    {
      auto cfg = _bus.config();
      cfg.spi_host = SPI2_HOST;  
      cfg.spi_mode = 0;
      cfg.freq_write = 80000000;
      cfg.freq_read  = 16000000;
      cfg.pin_sclk = 4;
      cfg.pin_mosi = 6;
      cfg.pin_miso = -1;
      cfg.pin_dc   = 19;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs   = 18;
      cfg.pin_rst  = 20;
      cfg.pin_busy = -1;
      cfg.memory_width  = 240;
      cfg.memory_height = 240;
      cfg.panel_width  = 240;
      cfg.panel_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.invert = false;
      cfg.rgb_order = false;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};