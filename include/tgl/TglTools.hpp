#ifndef TGL_TGLTOOLS_H
#define TGL_TGLTOOLS_H
#include <chrono>


#ifndef TGL_CHRONO_START
#define TGL_CHRONO_START TGL_CHRONO_START_LINE = __LINE__; TGL_CHRONO_START_TIME = std::chrono::high_resolution_clock::now(); 
#endif


#ifndef TGL_CHRONO_STOP
#define TGL_CHRONO_STOP TGL_CHRONO_STOP_TIME = std::chrono::high_resolution_clock::now(); std::cout << "|------------|\n| tgl_chrono |\n|------------|\n" << "file: "<< __FILE__ << "\nlines: "<< TGL_CHRONO_START_LINE << "-"<< __LINE__ << "\ntime_elapsed: "<< std::chrono::duration_cast<std::chrono::milliseconds>(TGL_CHRONO_STOP_TIME - TGL_CHRONO_START_TIME).count() << " ms\n\n"; TGL_CHRONO_COUNT++;
#endif

namespace tgl
{

static int TGL_CHRONO_COUNT = 0;
static int TGL_CHRONO_START_LINE = 0;
static std::chrono::high_resolution_clock::time_point TGL_CHRONO_START_TIME, TGL_CHRONO_STOP_TIME;

} // End of namespace tgl
#endif //TGL_TGLTOOLS_H
