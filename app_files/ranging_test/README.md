# LOCALIZATION_TEST Description

This is a basic script for querying range data from openmote periodically.

The mbed first requests the range data from the openmote by sending a msg of type SOUND_RANGE_REQ. 
The range request contains a range_param_t which specifies the mode to range in and the number of 
samples to take. 

The openmote interprets this message and sends the data back down as a series of packets filled 
with a range_hdr_t struct. 

The range_hdr_t struct contains a flag to signify if it is the last in as series of packets and 
an array of range_data_t structs.

The range_data_t struct contains the Time Difference of Arrival (TDoA) and the Orientation 
Difference of Arrival (OD) and an error flag in case a pin had missed a ping.

