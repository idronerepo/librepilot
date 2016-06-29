#include "data_sets.h"
#include "stddef.h"

#if defined(__GNUC__)
#define SWAP32 __builtin_bswap32
#elif defined(__ICCAVR32__)
#define SWAP32 __swap_bytes
#elif defined(_WIN32)
#include "intrin.h"
#define SWAP32 _byteswap_ulong
#else
#define SWAP16(v) ((uint16_t)(((uint16_t)(v) >> 8) | ((uint16_t)(v) << 8)))
#define SWAP32(v) ((uint32_t)(((uint32_t)SWAP16((uint32_t)(v) >> 16)) | ((uint32_t)SWAP16((uint32_t)(v)) << 16)))
#endif

#ifndef OFFSETOF
#define OFFSETOF offsetof//(TYPE, MEMBER) ((uint8_t)&(((TYPE*)0)->MEMBER))
#endif

unsigned char IS_LITTLE_ENDIAN = 0xFF;

// Reversed upper and lower 32 bit words in a double.
// change to 32 bit compiler if you are getting errors here.
// compiler will likely inline this as it's a tiny function
void flipDouble( uint8_t* ptr )
{
	const uint32_t* w = (const uint32_t *)(ptr);
	union
	{
		double          v;
		uint32_t        w[2];
	} u;

	u.w[0] = w[1];
	u.w[1] = w[0];

	*(double*)ptr = u.v;
}

int initDataSets(void)
{
	union
	{
		uint8_t  c[4];
		uint32_t i;
	} u;

	u.i = 0x01020304;

	if (0x04 == u.c[0])
	{
		IS_LITTLE_ENDIAN = 1;
	}
	else if (0x01 == u.c[0])
	{
		IS_LITTLE_ENDIAN = 0;
	}
	else
	{
		return 0;
	}
	
	return 1;
}

void flipEndianess32(uint8_t* data, int dataLength)
{
	// data must be 4 byte aligned to swap endian-ness
	if (dataLength & 0x00000003)
	{
		return;
	}
	
	uint32_t* dataPtr = (uint32_t*)data;
	uint32_t* dataPtrEnd = (uint32_t*)(data + dataLength);
	uint32_t tmp;
	while (dataPtr < dataPtrEnd)
	{
		tmp = *dataPtr;
		*dataPtr++ = SWAP32(tmp);
	}
}

void flipDoubles(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength)
{
	uint16_t* doubleOffsetsEnd = offsets + offsetsLength;
	int offsetToDouble;
	int maxDoubleOffset = dataLength - 8;

	while (offsets < doubleOffsetsEnd)
	{
		offsetToDouble = (*offsets++) - offset;
		if (offsetToDouble >= 0 && offsetToDouble <= maxDoubleOffset)
		{
			flipDouble(data + offsetToDouble);
		}
	}
}

void flipStrings(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength)
{
	uint16_t* stringOffsetsEnd = offsets + offsetsLength;
	int offsetToString;
	int lengthOfString;
	int maxStringOffset;

	while (offsets < stringOffsetsEnd)
	{
		offsetToString = (*offsets++) - offset;
		lengthOfString = (*offsets++);
		maxStringOffset = dataLength - lengthOfString;
		if (offsetToString >= 0 && offsetToString <= maxStringOffset)
		{
			flipEndianess32(data + offsetToString, lengthOfString);
		}
	}
}

#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable: 4267)

#endif

uint16_t* getDoubleOffsets(int dataId, uint16_t* offsetsLength)
{
	// first offset is the number of offsets
	static uint16_t offsetsImu[] =
	{
		1,
		OFFSETOF(imu_t, time)
	};

	static uint16_t offsetsIns1[] =
	{
		4,
		OFFSETOF(ins_1_t, timeOfWeek),
		OFFSETOF(ins_1_t, lla[0]),
		OFFSETOF(ins_1_t, lla[1]),
		OFFSETOF(ins_1_t, lla[2])
	};

	static uint16_t offsetsIns2[] =
	{
		4,
		OFFSETOF(ins_2_t, timeOfWeek),
		OFFSETOF(ins_2_t, lla[0]),
		OFFSETOF(ins_2_t, lla[1]),
		OFFSETOF(ins_2_t, lla[2])
	};

	static uint16_t offsetsGps[] =
	{
		4,
		OFFSETOF(gps_t, pos.lla[0]),
		OFFSETOF(gps_t, pos.lla[1]),
		OFFSETOF(gps_t, pos.lla[2]),
		OFFSETOF(gps_t, towOffset)
	};

	static uint16_t offsetsGpsPos[] =
	{
		3,
		OFFSETOF(gps_nav_poslla_t, lla[0]),
		OFFSETOF(gps_nav_poslla_t, lla[1]),
		OFFSETOF(gps_nav_poslla_t, lla[2])
	};

	static uint16_t offsetsInsMisc[] =
	{
		4,
		OFFSETOF(ins_misc_t, timeOfWeek),
		OFFSETOF(ins_misc_t, x.lla[0]),
		OFFSETOF(ins_misc_t, x.lla[1]),
		OFFSETOF(ins_misc_t, x.lla[2]),
	};

	static uint16_t offsetsInsRes[] =
	{
		3,
		OFFSETOF( ins_res_t, x_dot.lla[0] ),
		OFFSETOF( ins_res_t, x_dot.lla[1] ),
		OFFSETOF( ins_res_t, x_dot.lla[2] ),
	};

	static uint16_t offsetsSysSensors[] =
	{
		1,
		OFFSETOF(sys_sensors_t, time)
	};
	
	static uint16_t offsetsFlashConfig[] =
	{
		6,
		OFFSETOF( nvm_flash_cfg_t, refLla[0] ),
		OFFSETOF( nvm_flash_cfg_t, refLla[1] ),
		OFFSETOF( nvm_flash_cfg_t, refLla[2] ),
		OFFSETOF( nvm_flash_cfg_t, lastLla[0] ),
		OFFSETOF( nvm_flash_cfg_t, lastLla[1] ),
		OFFSETOF( nvm_flash_cfg_t, lastLla[2] )
	};

    static uint16_t* doubleOffsets[DID_EXTERNAL_COUNT] =
	{
		0,						// DID_NULL
		0,						// DID_DEV_INFO
		offsetsImu,				// DID_IMU
		offsetsImu,				// DID_CON_SCUL_INT
		offsetsIns1,			// DID_INS_1
		offsetsIns2,			// DID_INS_2
		offsetsGps,				// DID_GPS
		0,						// DID_CONFIG
		0,						// DID_ASCII_BCAST_PERIOD
		offsetsInsMisc,			// DID_INS_MISC
		0,						// DID_SYS_PARAMS
		offsetsSysSensors,		// DID_SYS_SENSORS
		offsetsFlashConfig,		// DID_FLASH_CONFIG
		0,						// DID_GPS_RSSI
		offsetsGpsPos,			// DID_GPS_POS
		0,						// DID_GPS_VEL
		0,						// DID_IO
		0,						// DID_IO_SERVOS_PWM
		0,						// DID_IO_SERVOS_PPM
		0,						// DID_MAGNETOMETER_CAL
		offsetsInsRes,			// DID_INS_RESOURCES
        0,                      // DID_DGPS_CORRECTION
        0                       // DID_RTK
	};

	// protect from buffer over-run
    if (DID_IS_EXTERNAL(dataId))
	{
        dataId = DID_FLATTEN_EXTERNAL(dataId);

        uint16_t* offsets;
        if ((offsets = doubleOffsets[dataId]))
        {
            *offsetsLength = (*offsets++);
            return offsets;
        }
    }
	
	return 0;
}

#ifdef _MSC_VER

#pragma warning(pop)

#endif

uint16_t* getStringOffsetsLengths(int dataId, uint16_t* offsetsLength)
{
	static uint16_t devInfoOffsets[] =
	{
		4,	// 4x number of strings
		OFFSETOF( dev_info_t, manufacturer ), sizeof( ((dev_info_t*)0)->manufacturer ) / sizeof( ((dev_info_t*)0)->manufacturer[0] )
	};

    static uint16_t* stringOffsets[DID_EXTERNAL_COUNT] =
	{
		0,						// DID_NULL
		devInfoOffsets,			// DID_DEV_INFO
		0,						// DID_IMU
		0,						// DID_CON_SCUL_INT
		0,						// DID_INS_1
		0,						// DID_INS_2
		0,						// DID_GPS
		0,						// DID_CONFIG
		0,						// DID_ASCII_BCAST_PERIOD
		0,						// DID_INS_MISC
		0,						// DID_SYS_PARAMS
		0,						// DID_SYS_SENSORS
		0,						// DID_FLASH_CONFIG
		0,						// DID_GPS_RSSI
		0,						// DID_GPS_POS
		0,						// DID_GPS_VEL
		0,						// DID_IO
		0,						// DID_IO_SERVOS_PWM
		0,						// DID_IO_SERVOS_PPM
		0,						// DID_MAGNETOMETER_CAL
		0,						// DID_INS_RESOURCES
        0,                      // DID_DGPS_CORRECTION
        0                       // DID_RTK
	};

	// protect from buffer over-run
    if (DID_IS_EXTERNAL(dataId))
	{
        dataId = DID_FLATTEN_EXTERNAL(dataId);

        uint16_t* offsets;
        if ((offsets = stringOffsets[dataId]))
        {
            *offsetsLength = (*offsets++);
            return offsets;
        }
    }

	return 0;
}

uint32_t checksum32(void* data, int count);
uint32_t checksum32(void* data, int count)
{
	if (count % 4 != 0)
	{
		return 0;
	}
	
	uint32_t checksum = 0;
	uint32_t* dataPtr = (uint32_t*)data;
	uint32_t* dataEnd = dataPtr + (count / 4);
	
	while (dataPtr < dataEnd)
	{
		checksum ^= *dataPtr++;
	}
	
	return checksum;
}

// This function skips the first 4 bytes (one 4 byte word), which are assumed to be the checksum in the serial number flash memory data structure.
uint32_t serialNumChecksum32(void* data, int size)
{
	return checksum32( (uint8_t*)data + 4, size - 4 );
}

// This function skips the first 8 bytes (two 4 byte words), which are assumed to be the size and checksum in flash memory data structures.
uint32_t flashChecksum32(void* data, int size)
{
	return checksum32( (uint8_t*)data + 8, size - 8 );
}

#if RTK_EMBEDDED

static void flipRTKObservation(obsd_t* d)
{
	// we don't flip the endianess of obsd_t because it contains char data
	d->time.time = SWAP32(d->time.time);
	flipEndianess32((uint8_t*)&d->time.sec, sizeof(double));
	flipDouble((uint8_t*)&d->time.sec);
	flipEndianess32((uint8_t*)d->D, sizeof(d->D[0]) * sizeof(d->D));
	flipEndianess32((uint8_t*)d->L, sizeof(d->L[0]) * sizeof(d->L));
	flipEndianess32((uint8_t*)d->P, sizeof(d->P[0]) * sizeof(d->P));
	for (int i = 0; i < NFREQ + NEXOBS; i++)
	{
		flipDouble((uint8_t*)&d->L[i]);
		flipDouble((uint8_t*)&d->P[i]);
	}
}

static void flipRTKEphemeris(eph_t* e)
{
	flipEndianess32((uint8_t*)e, sizeof(eph_t));
	flipDouble((uint8_t*)&e->toe.sec);
	flipDouble((uint8_t*)&e->toc.sec);
	flipDouble((uint8_t*)&e->ttr.sec);
	flipDouble((uint8_t*)&e->A);
	flipDouble((uint8_t*)&e->e);
	flipDouble((uint8_t*)&e->i0);
	flipDouble((uint8_t*)&e->OMG0);
	flipDouble((uint8_t*)&e->omg);
	flipDouble((uint8_t*)&e->M0);
	flipDouble((uint8_t*)&e->deln);
	flipDouble((uint8_t*)&e->OMGd);
	flipDouble((uint8_t*)&e->idot);
	flipDouble((uint8_t*)&e->crc);
	flipDouble((uint8_t*)&e->crs);
	flipDouble((uint8_t*)&e->cuc);
	flipDouble((uint8_t*)&e->cus);
	flipDouble((uint8_t*)&e->cic);
	flipDouble((uint8_t*)&e->cis);
	flipDouble((uint8_t*)&e->toes);
	flipDouble((uint8_t*)&e->fit);
	flipDouble((uint8_t*)&e->f0);
	flipDouble((uint8_t*)&e->f1);
	flipDouble((uint8_t*)&e->f2);
	flipDouble((uint8_t*)&e->tgd[0]);
	flipDouble((uint8_t*)&e->tgd[1]);
	flipDouble((uint8_t*)&e->tgd[2]);
	flipDouble((uint8_t*)&e->tgd[3]);
	flipDouble((uint8_t*)&e->Adot);
	flipDouble((uint8_t*)&e->ndot);
}

static void flipRTKGlonassEphemeris(geph_t* g)
{
	int i;

	flipEndianess32((uint8_t*)g, sizeof(geph_t));
	flipDouble((uint8_t*)&g->toe.sec);
	flipDouble((uint8_t*)&g->tof.sec);
	for (i = 0; i < 3; i++)
	{
		flipDouble((uint8_t*)&g->pos[i]);
		flipDouble((uint8_t*)&g->vel[i]);
		flipDouble((uint8_t*)&g->acc[i]);
	}
	flipDouble((uint8_t*)&g->taun);
	flipDouble((uint8_t*)&g->gamn);
	flipDouble((uint8_t*)&g->dtaun);
}

static void flipRTKAntenna(antenna_t* a)
{
	int i;

	flipEndianess32((uint8_t*)a, sizeof(antenna_t));
	for (i = 0; i < 3; i++)
	{
		flipDouble((uint8_t*)&a->pos[i]);
		flipDouble((uint8_t*)&a->del[i]);
	}
	flipDouble((uint8_t*)&a->height);
}

void flipRTK(rtk_data_t* r)
{
	switch (r->dataType)
	{
	case rtk_data_type_observation:
		flipRTKObservation(&r->data.obsd);
		break;

	case rtk_data_type_rover_ephemeris:
		flipRTKEphemeris(&r->data.eph);
		break;

	case rtk_data_type_rover_glonass_ephemeris:
		flipRTKGlonassEphemeris(&r->data.geph);
		break;

	case rtk_data_type_base_station_antenna:
		flipRTKAntenna(&r->data.antenna);
		break;

	default:
		break;
	}
}

#endif
