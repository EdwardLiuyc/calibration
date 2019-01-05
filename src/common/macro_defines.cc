#include "simple_time.h"
#include <chrono>
#include <cstring>

static char filename[128] = { '\0' };
char *splited_file_name(const char *file)
{
  memset(filename, 0, sizeof(filename));
  std::string str = file;
  int index = str.find_last_of('/');
  strcpy(filename, str.substr(index + 1).c_str());

  return filename;
}

SimpleTime s_debug_start_time;
void start_clock() { s_debug_start_time = SimpleTime::get_current_time(); }

void end_clock( const char* filename, const char* func_name, const int line )
{
  // PRINT_DEBUG_FMT("Cost: %lf s",
  //                 (SimpleTime::get_current_time() - s_debug_start_time).toSec());
  PRINT_COLOR_FMT( BOLD, "[ %s: %s: %d ] Cost : %lf s", 
    splited_file_name(filename), func_name, line, 
    (SimpleTime::get_current_time() - s_debug_start_time).toSec());
}

void end_clock_min_time(double min_time)
{
  double spent_time =
    (SimpleTime::get_current_time() - s_debug_start_time).toSec();
  if (spent_time > min_time)
    PRINT_DEBUG_FMT("Cost: %lf s", spent_time);
}

// using c++ std
std::chrono::high_resolution_clock::time_point debug_start_time_std;
void start_clock_std()
{
  debug_start_time_std = std::chrono::high_resolution_clock::now();
}

void end_clock_std()
{
  std::chrono::high_resolution_clock::time_point end =
    std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::ratio<1, 1>> duration_s(
    end - debug_start_time_std);
  PRINT_DEBUG_FMT("Cost: %lf s", duration_s.count());
}

void end_clock_min_time_std(double min_time)
{
  std::chrono::high_resolution_clock::time_point end =
    std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::ratio<1, 1>> duration_s(
    end - debug_start_time_std);
  double spent_time = duration_s.count();
  if (spent_time > min_time)
    PRINT_DEBUG_FMT("Cost: %lf s", spent_time);
}
