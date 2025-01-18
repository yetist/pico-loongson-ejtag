
#include <tusb.h>

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;

  // connected
  if ( dtr && rts )
  {
    // print initial message when connected
    tud_cdc_write_str("\r\nTinyUSB WebUSB device example\r\n");
    tud_cdc_write_flush();
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;

  uint8_t buf[64];
  uint32_t count = tud_cdc_read(buf, sizeof(buf));

  // dump to console
  tud_cdc_write_str("- CDC RX:\r\n");
  tud_cdc_write_flush();
//   dump_binary_to_console(buf, count);
  tud_cdc_write_str("\r\n");
  tud_cdc_write_flush();
}
