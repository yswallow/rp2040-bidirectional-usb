#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

#include "pio_usb.h"

// Use tinyUSB header to define USB descriptors
#include "device/usbd.h"
#include "class/hid/hid_device.h"

#include "tusb.h"

static usb_device_t *usb_device = NULL;

tusb_desc_device_t const desc_device = {.bLength = sizeof(tusb_desc_device_t),
                                        .bDescriptorType = TUSB_DESC_DEVICE,
                                        .bcdUSB = 0x0110,
                                        .bDeviceClass = 0x00,
                                        .bDeviceSubClass = 0x00,
                                        .bDeviceProtocol = 0x00,
                                        .bMaxPacketSize0 = 64,

                                        .idVendor = 0xCafe,
                                        .idProduct = 0,
                                        .bcdDevice = 0x0100,

                                        .iManufacturer = 0x01,
                                        .iProduct = 0x02,
                                        .iSerialNumber = 0x03,

                                        .bNumConfigurations = 0x01};

enum {
  ITF_NUM_KEYBOARD,
  ITF_NUM_MOUSE,
  ITF_NUM_TOTAL,
};

enum {
  EPNUM_KEYBOARD = 0x81,
  EPNUM_MOUSE = 0x82,
};

#define PIO_DEVICE_PULLUP_PIN 4
#define PIO_POWERED_ENABLE_PIN 2
#define BUTTON_PIN 3

uint8_t const desc_hid_keyboard_report[] =
{
  TUD_HID_REPORT_DESC_KEYBOARD()
};

uint8_t const desc_hid_mouse_report[] =
{
  TUD_HID_REPORT_DESC_MOUSE()
};

const uint8_t *report_desc[] = {desc_hid_keyboard_report,
                                desc_hid_mouse_report};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 2*TUD_HID_DESC_LEN)
uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(ITF_NUM_KEYBOARD, 0, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(desc_hid_keyboard_report), EPNUM_KEYBOARD,
                       CFG_TUD_HID_EP_BUFSIZE, 10),
    TUD_HID_DESCRIPTOR(ITF_NUM_MOUSE, 0, HID_ITF_PROTOCOL_MOUSE,
                       sizeof(desc_hid_mouse_report), EPNUM_MOUSE,
                       CFG_TUD_HID_EP_BUFSIZE, 10),
};

static_assert(sizeof(desc_device) == 18, "device desc size error");

const char *string_descriptors_base[] = {
    [0] = (const char[]){0x09, 0x04},
    [1] = "Pico PIO USB",
    [2] = "Pico PIO USB Device",
    [3] = "123456",
};
static string_descriptor_t str_desc[4];

static void init_string_desc(void) {
  for (int idx = 0; idx < 4; idx++) {
    uint8_t len = 0;
    uint16_t *wchar_str = (uint16_t *)&str_desc[idx];
    if (idx == 0) {
      wchar_str[1] = string_descriptors_base[0][0] |
                     ((uint16_t)string_descriptors_base[0][1] << 8);
      len = 1;
    } else if (idx <= 3) {
      len = strnlen(string_descriptors_base[idx], 31);
      for (int i = 0; i < len; i++) {
        wchar_str[i + 1] = string_descriptors_base[idx][i];
      }

    } else {
      len = 0;
    }

    wchar_str[0] = (TUSB_DESC_STRING << 8) | (2 * len + 2);
  }
}

static usb_descriptor_buffers_t desc = {
    .device = (uint8_t *)&desc_device,
    .config = desc_configuration,
    .hid_report = report_desc,
    .string = str_desc
};

// tud config
uint8_t const * tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &desc_device;
}

uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance)
{
  switch(instance){
    case 0:
      return desc_hid_keyboard_report;
    case 1:
      return desc_hid_mouse_report;
    default:
      return desc_hid_keyboard_report;
  }
}

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}

char const* string_desc_arr [] =
{
  (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
  "TinyUSB",                     // 1: Manufacturer
  "TinyUSB Device",              // 2: Product
  "123456",                      // 3: Serials, should use chip ID
  "TinyUSB CDC",                 // 4: CDC Interface
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  if ( index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }else
  {
    // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
    // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = strlen(str);
    if ( chr_count > 31 ) chr_count = 31;

    // Convert ASCII string into UTF-16
    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}

// tud config end

uint8_t mouse_device = 0xFF;
uint8_t mouse_interface = 0xFF;

static bool native_device_mode = true;
void hid_task(void) {
  static uint32_t count = 0;
  count++;
  if( count==10000 ) {
    if( !gpio_get(BUTTON_PIN) ) {
      if(native_device_mode) {
        if(tud_hid_n_ready(1)) {
          tud_hid_n_mouse_report(1, 0, 0, -5, 0, 0, 0);
        }
      } else {
        // PIO device
        if (usb_device != NULL) {
          printf("EPNUM_MOUSE is ready\n");
          hid_mouse_report_t mouse_report = {0};
          mouse_report.x = 5;
          endpoint_t *ep = pio_usb_get_endpoint(usb_device, 2);
          pio_usb_set_out_data(ep, (uint8_t *)&mouse_report, sizeof(mouse_report));
        }
        else {
          printf("EPNUM_MOUSE is not ready\n");
        }
      }
    }
    if( mouse_device!=0xFF && mouse_interface!=0xFF ) {
      if( tuh_hid_receive_report(mouse_device, mouse_interface) ) {
        printf("report request!\r\n");
      } else
      {
        printf("report request fail\r\n");
      }
    }
      
    count = 0;
  }
}

void pio_host_core1_main() {
  ///sleep_ms(10);

  // To run USB SOF interrupt in core1, create alarm pool in core1.
  static pio_usb_configuration_t config = PIO_USB_DEFAULT_CONFIG;
  config.pin_dp = 6;
  config.alarm_pool = (void*)alarm_pool_create(2, 1);
  usb_device = pio_usb_host_init(&config);

  //// Call pio_usb_host_add_port to use multi port
  // const uint8_t pin_dp2 = 8;
  // pio_usb_host_add_port(pin_dp2);

  while (true) {
    pio_usb_host_task();
  }
}

void pio_device_core1_main() {
  gpio_init(PIO_DEVICE_PULLUP_PIN);
  gpio_set_dir(PIO_DEVICE_PULLUP_PIN, true);
  gpio_put(PIO_DEVICE_PULLUP_PIN, true);
  ///sleep_ms(10);

  static pio_usb_configuration_t config = PIO_USB_DEFAULT_CONFIG;
  config.pin_dp = 6;
  init_string_desc();
  usb_device = pio_usb_device_init(&config, &desc);

  while (true) {
    pio_usb_device_task();
  }
}

void leds_init(void) {
  gpio_init(25);
  gpio_init(16);
  gpio_init(17);

  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, false);
  gpio_pull_up(BUTTON_PIN);

  gpio_set_dir(25, true);
  gpio_set_dir(16, true);
  gpio_set_dir(17, true);

  // active low
  gpio_put(25, true);
  gpio_put(16, true);
  gpio_put(17, true);

  //gpio_init(26);
  gpio_init(PIO_POWERED_ENABLE_PIN);

  gpio_set_dir(PIO_POWERED_ENABLE_PIN, false);
  gpio_disable_pulls(PIO_POWERED_ENABLE_PIN);
  //gpio_set_dir(26, true);
  //gpio_put(26, false);
}

int main() {
  // default 125MHz is not appropreate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(144000, true);

  stdio_init_all();
  printf("hello!\n");

  leds_init();
  ///sleep_ms(10);
  multicore_reset_core1();
  sleep_ms(1); // to correct gpio_get()
  if(! gpio_get(PIO_POWERED_ENABLE_PIN)) {
    native_device_mode = true;
    // pio is host
    gpio_set_dir(PIO_POWERED_ENABLE_PIN, true);
    gpio_put(PIO_POWERED_ENABLE_PIN, true);
    //gpio_put(26, true);
    gpio_put(17, false);
    tud_init(0);
    // all USB task run in core1
    multicore_launch_core1(pio_host_core1_main);

    while (true) {
      if (usb_device != NULL) {
        for (int dev_idx = 0; dev_idx < PIO_USB_DEVICE_CNT; dev_idx++) {
          usb_device_t *device = &usb_device[dev_idx];
          if (!device->connected) {
            continue;
          }

          // Print received packet to EPs
          for (int ep_idx = 0; ep_idx < PIO_USB_DEV_EP_CNT; ep_idx++) {
            endpoint_t *ep = pio_usb_get_endpoint(device, ep_idx);

            if (ep == NULL) {
              break;
            }

            uint8_t temp[64];
            int len = pio_usb_get_in_data(ep, temp, sizeof(temp));

            if (len > 0) {
              printf("%04x:%04x EP 0x%02x:\t", device->vid, device->pid,
                    ep->ep_num);
              for (int i = 0; i < len; i++) {
                printf("%02x ", temp[i]);
              }
              printf("\n");
              if(len>=3) {
                // instance, reportid, buttons, x, y, wheel, horizontal
                tud_hid_n_mouse_report(1, 0, temp[0], temp[1], temp[2], 0, 0);
                gpio_put(25, false);
              }
            }
          }
        }
      }
      tud_task();
      hid_task();
      //sleep_us(1);
    }
  } else {
    native_device_mode = false;
    // powered from pio
    gpio_put(16, false);
    // pio is device
    //board_init();
    tuh_init(0);
    // all USB task run in core1
    multicore_launch_core1(pio_device_core1_main);

    while (true) {
      tuh_task();
      hid_task();
    }
  }
}

static void process_mouse_report(hid_mouse_report_t const* report) {
  // always PIO device mode
  if (usb_device != NULL) {
    endpoint_t *ep = pio_usb_get_endpoint(usb_device, 2);
    pio_usb_set_out_data(ep, (uint8_t *)report, sizeof(hid_mouse_report_t));
  }
}

// optional callbacks
void tuh_mount_cb(uint8_t dev_addr)
{
  // application set-up
  printf("A device with address %d is mounted\r\n", dev_addr);
}

void tuh_umount_cb(uint8_t dev_addr)
{
  // application tear-down
  printf("A device with address %d is unmounted \r\n", dev_addr);
  mouse_device = 0xFF;
}
// optional callbacks end

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
  printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);

  // Interface protocol (hid_interface_protocol_enum_t)
  const char* protocol_str[] = { "None", "Keyboard", "Mouse" };
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  printf("HID Interface Protocol = %s\r\n", protocol_str[itf_protocol]);
  if(itf_protocol==2) {
    mouse_device = dev_addr;
    mouse_interface = instance;
  }

  (void) desc_report;
  (void) desc_len;
}


void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) bufsize;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;
  return 0;
}


void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  gpio_put(25, false);
  switch (itf_protocol)
  {
    case HID_ITF_PROTOCOL_MOUSE:
      //TU_LOG2("HID receive boot mouse report\r\n");
      printf("HID receive boot mouse report\r\n");
      process_mouse_report( (hid_mouse_report_t const*) report);
      
    break;

    default:
      printf("HID Generic Report\r\n");
      if(len>=2) {
        gpio_put(16, false);
        process_mouse_report( (hid_mouse_report_t const*) report);
      }
    break;
  }

  // continue to request to receive report
  if ( !tuh_hid_receive_report(dev_addr, instance) )
  {
    printf("Error: cannot request to receive report\r\n");
  }
}
