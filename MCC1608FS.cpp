/************************************************************/
/*    NAME: Nick Rypkema                                    */
/*    ORGN: MIT                                             */
/*    FILE: MCC1608FS.cpp                                   */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MCC1608FS.h"
#include <bot_param/param_util.h>
#include <ConciseArgs>

#include "lcmtypes/MCC1608FSlcm/scan_t.hpp"

using namespace std;
using namespace boost;
using namespace boost::posix_time;
using namespace MCC1608FSlcm;

// Signal handler to catch ctrl-c and exit driver cleanly
volatile sig_atomic_t flag = 0;
void SigHandler(int s)
{
  flag = 1;
}

//---------------------------------------------------------
// Constructor

MCC1608FS::MCC1608FS()
{
}

//---------------------------------------------------------
// Destructor

MCC1608FS::~MCC1608FS()
{
  if (m_connected) {
    usbAInScanStop_USB1608FS_Plus(m_usb_handle);
    usbReset_USB1608FS_Plus(m_usb_handle);
    cleanup_USB1608FS_Plus(m_usb_handle);
    sleep(2);
  }
}

void MCC1608FS::Run()
{
  while(1) {
    if (flag) {
      cout << "DAQ driver exiting..." << endl;
      usbAInScanStop_USB1608FS_Plus(m_usb_handle);
      usbReset_USB1608FS_Plus(m_usb_handle);
      cleanup_USB1608FS_Plus(m_usb_handle);
      sleep(2);
      exit(1);
    }

    ptime pti0 = microsec_clock::local_time();  // time how long this read takes

    // get data
    usbAInScanStart_USB1608FS_Plus(m_usb_handle, m_count, m_freq_sampling, m_channels, m_options);
    m_ret_success = usbAInScanRead_USB1608FS_Plus(m_usb_handle, 1, m_num_chan, m_ret_analog_data, m_options);

    FILE* savefile;
    if (m_write_files) {
      // setup filepath
      stringstream filepath;
      // set timestamp
      clock_gettime(CLOCK_REALTIME,&m_tp);
      m_timestamp_s = (m_tp).tv_sec * 1e9;
      m_timestamp = (m_tp).tv_nsec + m_timestamp_s;
      filepath << m_write_path << m_timestamp << ".txt";
      savefile = fopen(filepath.str().c_str(), "w");
    }

    // check results
    printf("number of samples read = %d\n", m_ret_success/2);
    if (m_ret_success != (int)m_count*m_num_chan*2) {
      printf("ERROR: m_ret_success (total reads) = %d | m_count (reads per channel) = %d | m_num_chan (num channels) = %d\n", m_ret_success/2, m_count, m_num_chan);
      m_error=true;
    } else {
      for (unsigned int i = 0; i < m_count; i++) {
        for (unsigned int j = 0; j < m_num_chan; j++)	{
          int k = i*m_num_chan + j;
          m_data = rint(m_ret_analog_data[k]*m_table_AIN[m_range][j][0] + m_table_AIN[m_range][j][1]);
          m_voltage = volts_USB1608FS_Plus(m_data, m_range);
          m_voltage_data[i][j] = m_voltage;
          if (m_write_files) {
            if (j == 0) {
              fprintf(savefile, "%f", m_voltage);
            } else {
              fprintf(savefile, ",%f", m_voltage);
            }
          }
        }
        if (m_write_files) {
          fprintf(savefile, "\n");
        }
      }
      if (m_write_files) {
        fclose (savefile);
      }

      //set publish time stamp
      clock_gettime(CLOCK_REALTIME,&m_tp);
      m_timestamp_s = (m_tp).tv_sec * 1e9;
      m_timestamp = (m_tp).tv_nsec + m_timestamp_s;

      scan_t output_msg;
      output_msg.timestamp = m_timestamp;
      output_msg.num_samples = m_num_samples;
      output_msg.num_channels = m_num_channels;
      output_msg.scan_size = m_num_samples*m_num_channels;
      m_voltage_data_pointer = (float*) m_voltage_data.data();
      output_msg.scan = std::vector<double> (m_voltage_data_pointer, m_voltage_data_pointer + m_voltage_data.num_elements());
      m_lcm.publish("MCC1608FS_scan", &output_msg);

      ptime pti1 = microsec_clock::local_time();
      time_duration pdiffid=pti1-pti0;
      cout << "total read took: " << pdiffid << " seconds" << endl;
    }
  }
}

bool MCC1608FS::Setup()
{
  // initialize DAQ
  int ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }
  m_usb_handle = NULL;
  m_ranges[0] = 0;
  m_ranges[1] = 0;
  m_ranges[2] = 0;
  m_ranges[3] = 0;
  m_ranges[4] = 0;
  m_ranges[5] = 0;
  m_ranges[6] = 0;
  m_ranges[7] = 0;
  m_options = 4;    //00000100=4 select trigger on leading edge
  m_count = (uint32_t)m_num_samples;
  m_num_chan = (int)m_num_channels;
  m_ret_analog_data = new uint16_t[m_num_chan*m_count];
  m_voltage_data.resize(boost::extents[m_num_samples][m_num_channels]);
  m_error = false;

  // attempt to connect to DAQ
  if ((m_usb_handle = usb_device_find_USB_MCC(USB1608FS_PLUS_PID, NULL))) {
    printf("Success! USB mcc-1608FS-Plus found and connected!\n");
    m_connected = true;
  } else {
    printf("Failure! Could not find a connected USB mcc-1608FS-Plus!\n");
    m_connected = false;
    return false;
  }
  usbBuildGainTable_USB1608FS_Plus(m_usb_handle, m_table_AIN);

  usbAInScanStop_USB1608FS_Plus(m_usb_handle);  // clear the buffer

  // build bitmap for the first m_num_chan in m_channels.
  m_channels = 0;
  for (unsigned int i = 0; i < m_num_chan; i++) {
    m_channels |= (1 << i);
  }
  printf ("channels: %02X count:%d\n", m_channels, m_count);
  memset(m_ranges, m_range, sizeof(m_ranges));

  usbAInScanConfig_USB1608FS_Plus(m_usb_handle, m_ranges);  // configure for ranges

  m_sig_int_handler.sa_handler = SigHandler;
  sigemptyset(&m_sig_int_handler.sa_mask);
  m_sig_int_handler.sa_flags = 0;
  sigaction(SIGINT, &m_sig_int_handler, NULL);
  sigaction(SIGTERM, &m_sig_int_handler, NULL);

  if(!m_lcm.good())
    return false;

  return true;
}

int main(int argCount, char** argValues)
{
  ConciseArgs parser(argCount, argValues);
  std::string config_filename;
  parser.add(config_filename, "c", "config", "configuration file");
  parser.parse();

  MCC1608FS daq = MCC1608FS();

  BotParam* bot_param = bot_param_new_from_file(config_filename.c_str());

  daq.m_num_samples = bot_param_get_int_or_fail(bot_param, "drivers.MCC1608FS.num_samples");
  std::cout << "Number of samples: " << daq.m_num_samples << std::endl;
  daq.m_num_channels = bot_param_get_int_or_fail(bot_param, "drivers.MCC1608FS.num_channels");
  std::cout << "Number of channels: " << daq.m_num_channels << std::endl;
  daq.m_freq_sampling = bot_param_get_double_or_fail(bot_param, "drivers.MCC1608FS.freq_sampling");
  std::cout << "Sampling Frequency: " << daq.m_freq_sampling << std::endl;
  daq.m_write_files = bot_param_get_boolean_or_fail(bot_param, "drivers.MCC1608FS.write_files");
  std::cout << "File writing on: " << daq.m_write_files << std::endl;
  daq.m_voltage_range = bot_param_get_double_or_fail(bot_param, "drivers.MCC1608FS.voltage_range");
  std::cout << "Voltage range: " << daq.m_voltage_range << std::endl;
  if (daq.m_voltage_range == 1.0) {
    daq.m_range = BP_1V;  // 1V range (supposedly the officially supported min voltage range)
  } else if (daq.m_voltage_range == 1.25) {
    daq.m_range = BP_1_25V;
  } else if (daq.m_voltage_range == 2.0) {
    daq.m_range = BP_2V;
  } else if (daq.m_voltage_range == 2.5) {
    daq.m_range = BP_2_5V;
  } else if (daq.m_voltage_range == 5.0) {
    daq.m_range = BP_5V;
  } else if (daq.m_voltage_range == 10.0) {
    daq.m_range = BP_10V;
  } else {
    std::cerr << "VOLTAGE_RANGE not valid! Must be one of [1.0, 1.25, 2.0, 2.5, 5.0, 10.0]. Defaulting to 1.0..." << std::endl;
    daq.m_range = BP_1V;
  }
  if (daq.m_write_files) {
    daq.m_write_path = bot_param_get_str_or_fail(bot_param, "drivers.MCC1608FS.write_path");
    std::cout << "Write path: " << daq.m_write_path << std::endl;
  }

  if (daq.Setup()) {
    daq.Run();
  }

  return(0);
}

