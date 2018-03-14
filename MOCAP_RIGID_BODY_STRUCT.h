
#ifndef MOCAP_RIGID_BODY_H_
#define MOCAP_RIGID_BODY_H_


typedef struct MOCAP_RIGID_BODY_STRUCT
{
	int   m_ID;
	float m_timestamp;
	float m_qx;
	float m_qy;
	float m_qz;
	float m_qw;
	float m_x;
	float m_y;
	float m_z;

};
		
extern MOCAP_RIGID_BODY_STRUCT MOCAP_DATA;

#endif

