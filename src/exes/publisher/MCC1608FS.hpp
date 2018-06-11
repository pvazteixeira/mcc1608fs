/************************************************************/
/*    NAME: Nick Rypkema                                    */
/*    ORGN: MIT                                             */
/*    FILE: MCC1608FS.h                                     */
/*    DATE:                                                 */
/************************************************************/

#ifndef MCC1608FS_HEADER
#define MCC1608FS_HEADER

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include <sys/types.h>
#include <asm/types.h>
#include <stdbool.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/multi_array.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include <mcc1608fs/pmd.h>
#include <mcc1608fs/usb-1608FS-Plus.h>
#include <lcm/lcm-cpp.hpp>

class MCC1608FS
{
 public:
   MCC1608FS();
   ~MCC1608FS();
   void Run();
   bool Setup();

 public: // Configuration variables
   unsigned int     m_num_samples;
   double           m_freq_sampling;
   unsigned int     m_num_channels;
   bool             m_write_files;
   std::string      m_write_path;
   double           m_voltage_range;
   uint8_t          m_range;

 private: // State variables
   bool             m_connected;
   libusb_device_handle*  m_usb_handle;
   float            m_table_AIN[NGAINS_USB1608FS_PLUS][NCHAN_USB1608FS_PLUS][2];
   uint8_t          m_channel;
   uint8_t          m_channels;
   uint8_t          m_ranges[8];
   uint8_t          m_options;
   uint32_t         m_count;
   int              m_ret_success;
   uint16_t*        m_ret_analog_data;
   uint16_t         m_data;
   std::vector<double> m_voltage_data;
   double           m_voltage;
   int              m_num_chan;
   bool             m_error;
   struct sigaction m_sig_int_handler;
   timespec         m_tp;
   long long        m_timestamp_s;
   long long        m_timestamp;
   lcm::LCM         m_lcm;
};

#endif

