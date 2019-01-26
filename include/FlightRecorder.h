#ifndef FLIGHT_RECORDER_H_
#define FLIGHT_RECORDER_H_

/*
This class handles requesting data to be recorded. It does not have access to the
raw data write, but requests to write buffer of data when necessary. It also contains
the logic to 'decompile' the saved data when the host reqests it
*/

#include <DataStorageParameters.h>
#include <DataTagDeclerations.h>
#include <header.h>
#include <UpdateScheduler.h>

#define FLIGHT_RECORDER_DEFAULT_ACCEL_RECORD_FREQUENCY                      5
#define FLIGHT_RECORDER_DEFUALT_ANGULAR_RATE_RECORD_FREQUENCY               5
#define FLIGHT_RECORDER_DEFAULT_MAGNETIC_FIELD_RECORD_FREQUENCY             2
#define FLIGHT_RECORDER_DEFAULT_EULER_RECORD_FREQUENCY                      5
#define FLIGHT_RECORDER_DEFAULT_PRESSURE_RECORD_FREQUENCY                   4
#define FLIGHT_RECORDER_DEFUALT_PRESSURE_ALTITUDE_RECORD_FREQUENCY          6
#define FLIGHT_RECORDER_DEFAULT_TEMPERATURE_RECORD_FREQUENCY                2
#define FLIGHT_RECORDER_DEFAULT_GPS_RECORD_FREQUENCY                        1


using namespace RocketHelper;

class FlightRecorder{
public:
  int init();
  /*
  update
  @parameter updatedState: The new data the rocket has determined.
  @return: int >0 = storage request in bytes, 0 = no storage request

  stores any new data and if it is determined that data needs to be written, flags that data peice
  */
  int update(Rocket_State *updatedState);

  /*
  getRequestedStorageBuffer
  @param int *target buffer: target buffer to store data in
  @param uint maxSize: maximize data allowed in the target buffer
  @return int: amount of data written to the buffer. if it is negative, it is requesting another storage round

  parses the flagged data (see update()) and compiles into bytes and then stores in the requested
  array.
  */
  int getRequestedDataToStore(uint8_t *targetBuffer, uint maxSize);

  /*
  hardSetState
  'resets' the flight recorder's believed state of the rocket. Doing so will not cause any special
  flags to be written to storage. Used when going into simulation mode, as to 'trick' the rocket into
  acting like it's not.

  &param Rocket_State hardState: The state of the rocket to base all storage data off of.

  */
  int hardSetState(Rocket_State *hardState);

  /*
  translateData
  reads data, presumably from the flash storage, and translates it to be sent to the host.
  If a full data set is not available, does not attempt to read and will return the max index it
  parsed up to.

  @param uint8_t *dataToRead: stored data that needs to be translated from 8 bit format
  @param uint length: max length to be read
  @param float *target: array to store the parsed data into. size should be large enough to handle this
  @param uint targetLen: maximum length of target. Note, this should be sufficiently large to handle any data

  @return int: > 0 = how much data stored in target, -1 means more data required, -2 means targetLen not large enough

  */
  int translateNext(uint8_t *dataToRead, uint length, float *target, uint targetLen);


  float decodeAndScaleData(byte *buff, int byteLength, int scale, bool dataSigned = true);

  int scaleAndEncodeData(float data, uint numBytesToFill, int scale, byte*toFill);
  int determineEncodingByteSize(int dataFlag);
  int encodeData(int dataFlag, long timeStamp, float *data, uint8_t *target);

private:
  //saved state of the rocket, used to compare flags against
  Rocket_State runningState;
  //bools used to tell getRequestedDataToStore what data needs to be updated
  bool newIMUAccelData = false;
  bool newIMUAngularRateData = false;
  bool newIMUMagneticFieldData = false;
  bool newEulerData = false;
  bool newGPSData = false;
  bool newPressureData = false;
  bool newTemperatureData = false;
  bool newPressureAltitudeData = false;
  bool newFilteredAltitudeData = false;
  bool newFilteredVelocityData = false;
  bool newFilteredAccelerationsData = false;
  //special flag updates
  bool phaseChange = false;
  bool drogueChute = false;
  bool mainChute = false;

  //updaters used to determine when to update data
  //and yes, there's a lot....
  UpdateScheduler imuAccelUpdater, imuGyroUpdater, imuMagneticFieldUpdater, imuEulerUpdater, gpsUpdater,
  pressureUpdater, temperatureUpdater, pressureAltitudeUpdater, filteredAltitudeUpdater, filteredVelocityUpdater,
  filteredAccelerationUpdater;

  //functions



};

#endif
