#include "config.h"

GvdConfig::GvdConfig(){}

GvdConfig* GvdConfig::get(){
  static GvdConfig instance; 
  return &instance;
}
