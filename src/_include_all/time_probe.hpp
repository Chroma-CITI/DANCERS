#ifndef CPU_TIME_PROBE_HPP
#define CPU_TIME_PROBE_HPP

#include <time.h>
#include <fstream>
#include <chrono>

class CpuTimeProbe
{
public:
    CpuTimeProbe(void)
    {
        this->save_data = false;
    }

    CpuTimeProbe(std::string out_file)
    {
        this->save_data = true;
        this->out_file = out_file;

        // initialize the output file with headers
        file.open(out_file.c_str(), std::ios::out);
        file << "Time[us]" << std::endl;
        file.close();
    }

    /**
     * Start recording the CPU time
     */
    void start()
    {
        this->start_time = clock();
    }

    /**
     * Save the elapsed time since the last start() call. If save_data is true, it also writes it to the output file.
     */
    void stop()
    {
        this->end_time = clock();
        this->elapsed_time = float(end_time - start_time) / CLOCKS_PER_SEC;
        if (save_data)
        {
            file.open(out_file.c_str(), std::ios_base::app);
            file << get_elapsed_time() << std::endl;
            file.close();
        }
    }

    /**
     * Get the last saved elapsed time, in seconds.
     * 
     * @returns the elapsed time in seconds
     */
    float get_elapsed_time()
    {
        return elapsed_time;
    }
private:
    clock_t start_time, end_time;
    float elapsed_time;
    bool save_data;
    std::string out_file;
    std::ofstream file;
};

class WallTimeProbe
{
public:
    WallTimeProbe(void)
    {
        this->save_data = false;
    }

    WallTimeProbe(std::string out_file)
    {
        this->save_data = true;
        this->out_file = out_file;

        // initialize the output file with headers
        file.open(out_file.c_str(), std::ios::out);
        file << "Time[us]" << std::endl;
        file.close();
    }

    /**
     * Start recording the CPU time
     */
    void start()
    {
        this->start_time = std::chrono::system_clock::now();
    }

    /**
     * Save the elapsed time since the last start() call. If save_data is true, it also writes it to the output file.
     */
    void stop()
    {
        this->end_time = std::chrono::system_clock::now();
        this->elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();;
        if (save_data)
        {
            file.open(out_file.c_str(), std::ios_base::app);
            file << get_elapsed_time() << std::endl;
            file.close();
        }
    }

    /**
     * Get the last saved elapsed time, in seconds.
     * 
     * @returns the elapsed time in seconds
     */
    uint64_t get_elapsed_time()
    {
        return elapsed_time;
    }
private:
    std::chrono::system_clock::time_point start_time, end_time;
    int64_t elapsed_time;
    bool save_data;
    std::string out_file;
    std::ofstream file;
};

#endif // CPU_TIME_PROBE_HPP