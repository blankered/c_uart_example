
#ifndef MOCAP_RIGID_BODY_H_
#define MOCAP_RIGID_BODY_H_

class MOCAP_RIGID_BODY
{
	public:
		MOCAP_RIGID_BODY();
		~MOCAP_RIGID_BODY();

		void write_mocap_data(int, float, float, float, float, float, float, float);
	
		
	//public:
	//~MOCAP_RIGID_BODY();

		
	//{
	//	write_mocap_data(ID, qx, qy, qz, qw, x, y, z);;
	//}

/*	void write_mocap_data(int ID, float qx, float qy, float qz, float qw, float x, float y, float z)
	{
		m_ID = ID;
		m_qx = qx;
		m_qy = qy;
		m_qz = qz;
		m_qw = qw;
		m_x = x;
		m_y = y;
		m_z = z;
	}	*/	

	private:
		int   m_ID;
		float m_qx;
		float m_qy;
		float m_qz;
		float m_qw;
		float m_x;
		float m_y;
		float m_z;


};

//MOCAP_RIGID_BODY body;
#endif
