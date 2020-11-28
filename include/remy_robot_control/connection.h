#pragma once

#include <vector>
#include <mutex>
#include <atomic>

class Connection
{
  std::vector<unsigned char> data_;
  std::atomic<bool> opened_;
  std::mutex mutex;
  
  public:
    Connection() {
      data_.reserve(12);
    }
    ~Connection() {}

    int open(){
      opened_ = true;
    };
    
    int close(){
      opened_ = false;
    };

    bool isOpened() {
      return opened_;
    }

    int send(std::vector<unsigned char> &data){
      std::lock_guard<std::mutex>lock(mutex);
      data_ = data;
    };
    //send data to the robot. use explicit pointer convertion

    int receive(std::vector<unsigned char> &data){
      std::lock_guard<std::mutex>lock(mutex);
      data = data_;
    };
    //receive state of the robot. record to data. use explicit pointer convertion
};
