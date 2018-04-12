#ifndef HC_SR04_H
#define HC_SR04_H

#include <Arduino.h>

#define CM true
#define INCH false

class HC_SR04 {
  public:
    HC_SR04(int trigger, int echo, int interrupt, int max_dist=400);
    
    void begin();
    void start();
    bool isFinished(){ return _finished; }
    unsigned int getRange(bool units=CM);
    static HC_SR04* instance(){ return _instance; }
    
  private:
    static void _echo_isr();
    
    int _trigger, _echo, _int, _max;
    volatile unsigned long _start, _end;
    volatile bool _finished;
    static HC_SR04* _instance;
};
class HC_SR05 {
  public:
    HC_SR05(int trigger5, int echo5, int interrupt5, int max_dist5=400);
    
    void begin5();
    void start5();
    bool isFinished5(){ return _finished5; }
    unsigned int getRange5(bool units=CM);
    static HC_SR05* instance5(){ return _instance5; }
    
  private:
    static void _echo_isr5();
    
    int _trigger5, _echo5, _int5, _max5;
    volatile unsigned long _start5, _end5;
    volatile bool _finished5;
    static HC_SR05* _instance5;
};

#endif
