#include "action_script/ActionScript.h"

namespace ROBOTIS
{

ActionScript::ActionScript(ros::NodeHandle nh_, ros::NodeHandle param_nh_)
:nh(nh_), param_nh(param_nh_)
, DEBUG_PRINT(false)
, m_ActionFile(0)
, m_Playing(false)
, m_portNum(NotCount)
, m_startNode(false)
, manager_ready(false)
, joint_names({ "",
	"r_sho_pitch",
	"l_sho_pitch",
	"r_sho_roll",
	"l_sho_roll",
	"r_el","l_el",
	"r_hip_yaw",
	"l_hip_yaw",
	"r_hip_roll",
	"l_hip_roll",
	"r_hip_pitch",
	"l_hip_pitch",
	"r_knee",
	"l_knee",
	"r_ank_pitch",
	"l_ank_pitch",
	"r_ank_roll",
	"l_ank_roll",
	"head_pan",
	"head_tilt" })
, CENTER_VALUE(2048)
, MAX_VALUE(4095)
, MIN_ANGLE(-180.0) // degree
, MAX_ANGLE(180.0) // degree
, RATIO_VALUE2ANGLE(0.088) // 360 / 4096
, RATIO_ANGLE2VALUE(11.378) // 4096 / 360
{
	DEBUG_PRINT = true;
	// ros callback & publisher
	action_control_sub  = nh.subscribe("/play_motion", 100, &ActionScript::motionControlCallback, this);
	current_joint_sub   = nh.subscribe("/robot_joint_states", 100, &ActionScript::jointStatesCallback, this);
	controller_joint_pub = nh.advertise<sensor_msgs::JointState>("/controller_joint_states", 10);
	manager_ready_sub   = nh.subscribe("/manager_ready", 10, &ActionScript::manager_ready_callback, this);
	control_write_pub = nh_.advertise<robotis_controller::ControlWrite>("/control_write", 10);

	pp_pub = nh.advertise<robotis_controller::PublishPosition>("/publish_position", 10);
	ct_pub = nh.advertise<robotis_controller::ControlTorque>("/control_torque", 10);

	ros_thread = boost::thread(boost::bind(&ActionScript::ros_thread_proc, this));

	// Wait for robotis_manager initialization
	while(manager_ready == false)
		ros::spinOnce();
	sleep(2);

	// CM-740 control table address 24 : Dynamixel Power
	control_write(200, 24, 1, 1);
	usleep(100*1000);

	// Enable the torque of all dynamixels.
	control_write(254, 24, 1, 1);
	usleep(100*1000);

	/***** init pose start *****/
	// set the goal velocity of all dynamixels to 100
	control_write(254, 32, 2, 100);

	// init pose (sit down)
	sensor_msgs::JointState _init_joint_states;
	for(int i = 1; i < 21; i++)
		_init_joint_states.name.push_back(joint_names[i]);
	_init_joint_states.position.push_back(-0.840621);
	_init_joint_states.position.push_back( 0.719437);
	_init_joint_states.position.push_back(-0.328272);
	_init_joint_states.position.push_back( 0.360485);
	_init_joint_states.position.push_back( 0.509282);
	_init_joint_states.position.push_back(-0.518486);
	_init_joint_states.position.push_back(-0.007670);
	_init_joint_states.position.push_back(-0.023010);
	_init_joint_states.position.push_back( 0.013806);
	_init_joint_states.position.push_back(-0.007670);
	_init_joint_states.position.push_back(-1.182699);
	_init_joint_states.position.push_back( 1.148952);
	_init_joint_states.position.push_back( 2.247282);
	_init_joint_states.position.push_back(-2.265690);
	_init_joint_states.position.push_back( 1.219515);
	_init_joint_states.position.push_back(-1.239456);
	_init_joint_states.position.push_back( 0.044485);
	_init_joint_states.position.push_back(-0.016874);
	_init_joint_states.position.push_back( 0.003068);
	_init_joint_states.position.push_back( 0.191748);
	controller_joint_pub.publish(_init_joint_states);
	sleep(3);

	// set the goal velocity of all dynamixels to 0
	control_write(254, 32, 2, 0);
	/***** init pose end *****/

	// publish all joint position
	robotis_controller::PublishPosition _pp;
	for(int i = 1; i < 21; i++)
	{
		_pp.name.push_back(joint_names[i]);
		_pp.publish.push_back(true);
	}
	pp_pub.publish(_pp);

	ROS_INFO("Go Init.");
}

ActionScript::~ActionScript()
{
	if(m_ActionFile != 0)
		fclose(m_ActionFile);

	if(comm_thread.joinable() == true) comm_thread.join();
	if(ros_thread.joinable() == true) ros_thread.join();
}

void ActionScript::control_write(int id, int addr, int length, int value)
{
	robotis_controller::ControlWrite _cw;
	_cw.id = id;
	_cw.addr = addr;
	_cw.length = length;
	_cw.value = value;
	control_write_pub.publish(_cw);
}

void ActionScript::manager_ready_callback(const std_msgs::Bool::ConstPtr& msg)
{
	manager_ready = true;
}

void ActionScript::comm_thread_proc()
{
	ROS_INFO("Start action script processing..");
	ros::Rate _loop_hz(125);	// 8ms
	ros::Time _time;

	m_startNode = true;
	while(ros::ok())
	{
		// publish controller joint states
		// sensor_msgs::JointState _joint_states;

		// get desired current joint position
		if(getJointPosition() == true)
		{
			controller_joint_pub.publish(desired_position);
			desired_position.header.stamp = ros::Time::now();
		}
		_loop_hz.sleep();
	}
	ROS_INFO("Terminated ROS");
	return;
}

void ActionScript::ros_thread_proc()
{
	//	// wait init
	//	ROS_INFO("Wait start...");
	//	while(m_startNode == false) {}
	//
	//	// init motion
	//	Start(1);

	// ROS_INFO("Ready to play motion...");

	while(ros::ok())
		ros::spinOnce();
	return;
}

bool ActionScript::isJointNameInParam()
{
	int port_num = NoPort;
	if(nh.getParam("port_num", port_num) == false)
		return false;

	if(port_num < 1)
		return false;

	m_portNum = port_num;
	return true;
}


bool ActionScript::getJointPosition()
{
	sensor_msgs::JointState _joints;
	//////////////////// 吏��뿭 蹂��닔
	unsigned char bID;
	unsigned long ulTotalTime256T;
	unsigned long ulPreSectionTime256T;
	unsigned long ulMainTime256T;
	long lStartSpeed1024_PreTime_256T;
	long lMovingAngle_Speed1024Scale_256T_2T;
	long lDivider1,lDivider2;
	unsigned short wMaxAngle1024;
	unsigned short wMaxSpeed256;
	unsigned short wTmp;
	unsigned short wPrevTargetAngle; // Start position
	unsigned short wCurrentTargetAngle; // Target position
	unsigned short wNextTargetAngle; // Next target position
	unsigned char bDirectionChanged;

	///////////////// Static 蹂��닔
	static unsigned short wpStartAngle1024[NUMBER_OF_JOINTS]; // 蹂닿컙�븷 �떆�옉 吏��젏
	static unsigned short wpTargetAngle1024[NUMBER_OF_JOINTS]; // 蹂닿컙�븷 �룄李� 吏��젏
	static short int ipMovingAngle1024[NUMBER_OF_JOINTS]; // 珥� 媛��빞�븷 嫄곕━
	static short int ipMainAngle1024[NUMBER_OF_JOINTS]; // �벑�냽 援ш컙�뿉�꽌 媛��빞�븷 嫄곕━
	static short int ipAccelAngle1024[NUMBER_OF_JOINTS]; // 媛��냽 援ш컙�뿉�꽌 媛��빞�븷 嫄곕━
	static short int ipMainSpeed1024[NUMBER_OF_JOINTS]; // 紐⑺몴 �벑�냽�룄
	static short int ipLastOutSpeed1024[NUMBER_OF_JOINTS]; // �씠 �쟾 �긽�깭�쓽 �냽�룄(愿��꽦)
	static short int ipGoalSpeed1024[NUMBER_OF_JOINTS]; // 紐⑦꽣媛� �궡�빞 �븷 紐⑺몴�냽�룄
	static unsigned char bpFinishType[NUMBER_OF_JOINTS]; // �룄李� 吏��젏�뿉 �룄�떖�븷 �긽�깭
	short int iSpeedN;
	static unsigned short wUnitTimeCount;
	static unsigned short wUnitTimeNum;
	static unsigned short wPauseTime;
	static unsigned short wUnitTimeTotalNum;
	static unsigned short wAccelStep;
	static unsigned char bSection;
	static unsigned char bPlayRepeatCount;
	static unsigned short wNextPlayPage;

	/////////////// Enum 蹂��닔

	/**************************************
	 * Section             /----\
	 *                    /|    |\
	 *        /+---------/ |    | \
	 *       / |        |  |    |  \
	 * -----/  |        |  |    |   \----
	 *      PRE  MAIN   PRE MAIN POST PAUSE
	 ***************************************/

	if( m_Playing == false )
		return false;

	// ROS_INFO("Process 1");
	if( m_FirstDrivingStart == true ) // 泥섏쓬 �떆�옉�븷�븣
	{
		m_FirstDrivingStart = false; //First Process end
		m_PlayingFinished = false;
		m_StopPlaying = false;
		wUnitTimeCount = 0;
		wUnitTimeNum = 0;
		wPauseTime = 0;
		bSection = PAUSE_SECTION;
		m_PageStepCount = 0;
		bPlayRepeatCount = m_PlayPage.header.repeat;
		wNextPlayPage = 0;


		for( bID = 1; bID <= 20; bID++ )
		{
			// if(m_Joint.GetEnable(bID) == true)
			{
				for(int ix = 0; ix < 20; ix++)
				{
					std::string _name;
					if(getJointName(bID, _name) == false) continue;

					if(_name == current_joint_state.name[ix])
					{
						wpTargetAngle1024[bID] = Angle2Value( Rad2Deg(current_joint_state.position[ix]) );
					}
				}
				// wpTargetAngle1024[bID] = MotionStatus::m_CurrentJoints.GetValue(bID);
				ipLastOutSpeed1024[bID] = 0;
				ipMovingAngle1024[bID] = 0;
				ipGoalSpeed1024[bID] = 0;
			}
		}

	}
	// ROS_INFO("Process 2");
	if( wUnitTimeCount < wUnitTimeNum ) // �쁽�옱 吏꾪뻾以묒씠�씪硫�
	{
		wUnitTimeCount++;
		if( bSection == PAUSE_SECTION )
		{
			return false;
		}
		else
		{
			for( bID = 1; bID <= 20; bID++ )
			{
				std::string _name;
				if(getJointName(bID, _name) == false) continue;

				_joints.name.push_back(_name);
				double _deg = 0;

				// �쁽�옱 �궗�슜�븯�뒗 愿��젅留� 怨꾩궛
				//if(m_Joint.GetEnable(bID) == true)
				{
					if( ipMovingAngle1024[bID] == 0 )
					{
						_deg = Value2Angle(wpStartAngle1024[bID]);
						// m_Joint.SetValue(bID, wpStartAngle1024[bID]);
					}
					else
					{
						if( bSection == PRE_SECTION )
						{
							iSpeedN = (short)(((long)(ipMainSpeed1024[bID] - ipLastOutSpeed1024[bID]) * wUnitTimeCount) / wUnitTimeNum);
							ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;
							ipAccelAngle1024[bID] =  (short)((((long)(ipLastOutSpeed1024[bID] + (iSpeedN >> 1)) * wUnitTimeCount * 144) / 15) >> 9);

							//m_Joint.SetValue(bID, wpStartAngle1024[bID] + ipAccelAngle1024[bID]);

							_deg = Value2Angle(wpStartAngle1024[bID] + ipAccelAngle1024[bID]);
						}
						else if( bSection == MAIN_SECTION )
						{
							_deg = Value2Angle(wpStartAngle1024[bID] + (short int)(((long)(ipMainAngle1024[bID])*wUnitTimeCount) / wUnitTimeNum));

							ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
						}
						else // POST_SECTION
						{
							if( wUnitTimeCount == (wUnitTimeNum-1) )
							{
								// �뒪�뀦 留덉�留� �삤李⑤�� 以꾩씠湲곗쐞�빐 洹몃깷 紐⑺몴 �쐞移� 媛믪쓣 �궗�슜
								// m_Joint.SetValue(bID, wpTargetAngle1024[bID]);
								_deg = Value2Angle(wpTargetAngle1024[bID]);
							}
							else
							{
								if( bpFinishType[bID] == ZERO_FINISH )
								{
									iSpeedN = (short int)(((long)(0 - ipLastOutSpeed1024[bID]) * wUnitTimeCount) / wUnitTimeNum);
									ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;
									// m_Joint.SetValue(bID, wpStartAngle1024[bID] +  (short)((((long)(ipLastOutSpeed1024[bID] + (iSpeedN>>1)) * wUnitTimeCount * 144) / 15) >> 9));
									_deg = Value2Angle(wpStartAngle1024[bID] +  (short)((((long)(ipLastOutSpeed1024[bID] + (iSpeedN>>1)) * wUnitTimeCount * 144) / 15) >> 9));
								}
								else // NONE_ZERO_FINISH
								{
									// MAIN Section怨� �룞�씪�븯寃� �옉�룞-�룞�씪
									// step�뿉�꽌 �뼱�뼡�꽌蹂대뒗 媛�怨� �뼱�뼡 �꽌蹂대뒗 �꽌�빞�븯�뒗 �긽�솴�씠 諛쒖깮�븷 �닔 �엳�쑝誘�濡� �씠�젃寃� �븷 �닔諛뽰뿉 �뾾�쓬
									// m_Joint.SetValue(bID, wpStartAngle1024[bID] + (short int)(((long)(ipMainAngle1024[bID]) * wUnitTimeCount) / wUnitTimeNum));
									_deg = Value2Angle(wpStartAngle1024[bID] + (short int)(((long)(ipMainAngle1024[bID]) * wUnitTimeCount) / wUnitTimeNum));
									ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
								}
							}
						}
					}

					_joints.position.push_back(Deg2Rad(_deg));
					// m_Joint.SetSlope(bID, 1 << (m_PlayPage.header.slope[bID]>>4), 1 << (m_PlayPage.header.slope[bID]&0x0f));
					// m_Joint.SetPGain(bID, (256 >> (m_PlayPage.header.slope[bID]>>4)) << 2);
				}
			}
		}
		desired_position = _joints;
		return true;
	}
	else if( wUnitTimeCount >= wUnitTimeNum ) // �쁽�옱 Section�씠 �셿猷뚮릺�뿀�떎硫�
	{
		wUnitTimeCount = 0;

		for( bID = 1; bID <= 20; bID++ )
		{
			// if(m_Joint.GetEnable(bID) == true)
			{
				// wpStartAngle1024[bID] = m_Joint.GetValue(bID);
				for(int ix = 0; ix < 20; ix++)
				{
					std::string _name;
					if(getJointName(bID, _name) == false) continue;

					if(_name == current_joint_state.name[ix])
					{
						wpStartAngle1024[bID] = Angle2Value( Rad2Deg(current_joint_state.position[ix]) );
					}
				}
				ipLastOutSpeed1024[bID] = ipGoalSpeed1024[bID];
			}
		}

		// Section �뾽�뜲�씠�듃 ( PRE -> MAIN -> POST -> (PAUSE or PRE) ... )
		if( bSection == PRE_SECTION )
		{
			// MAIN Section 以�鍮�
			bSection = MAIN_SECTION;
			wUnitTimeNum =  wUnitTimeTotalNum - (wAccelStep << 1);

			for( bID = 1; bID <= 20; bID++ )
			{
				// if(m_Joint.GetEnable(bID) == true)
				{
					if( bpFinishType[bID] == NONE_ZERO_FINISH )
					{
						if( (wUnitTimeTotalNum - wAccelStep) == 0 ) // �벑�냽 援ш컙�씠 �쟾�� �뾾�떎硫�
							ipMainAngle1024[bID] = 0;
						else
							ipMainAngle1024[bID] = (short)((((long)(ipMovingAngle1024[bID] - ipAccelAngle1024[bID])) * wUnitTimeNum) / (wUnitTimeTotalNum - wAccelStep));
					}
					else // ZERO_FINISH
						ipMainAngle1024[bID] = ipMovingAngle1024[bID] - ipAccelAngle1024[bID] - (short int)((((long)ipMainSpeed1024[bID] * wAccelStep * 12) / 5) >> 8);
				}
			}
		}
		else if( bSection == MAIN_SECTION )
		{
			// POST Section 以�鍮�
			bSection = POST_SECTION;
			wUnitTimeNum = wAccelStep;

			for( bID = 1; bID <= 20; bID++ )
			{
				// if(m_Joint.GetEnable(bID) == true)
				ipMainAngle1024[bID] = ipMovingAngle1024[bID] - ipMainAngle1024[bID] - ipAccelAngle1024[bID];
			}
		}
		else if( bSection == POST_SECTION )
		{
			// Pause time �쑀臾댁뿉�뵲�씪 �떖�씪吏�
			if( wPauseTime )
			{
				bSection = PAUSE_SECTION;
				wUnitTimeNum = wPauseTime;
			}
			else
			{
				bSection = PRE_SECTION;
			}
		}
		else if( bSection == PAUSE_SECTION )
		{
			// PRE Section 以�鍮�
			bSection = PRE_SECTION;

			for( bID = 1; bID <= 20; bID++ )
			{
				// if(m_Joint.GetEnable(bID) == true)
				ipLastOutSpeed1024[bID] = 0;
			}
		}

		// PRE Section�떆�뿉 紐⑤뱺 以�鍮꾨�� �븳�떎.
		if( bSection == PRE_SECTION )
		{
			if( m_PlayingFinished == true ) // 紐⑥뀡�씠 �걹�궗�떎硫�
			{
				m_Playing = false;
				return false;
			}

			m_PageStepCount++;

			if( m_PageStepCount > m_PlayPage.header.stepnum ) // �쁽�옱 �럹�씠吏� �옱�깮�씠 �걹�궗�떎硫�
			{
				// �떎�쓬 �럹�씠吏� 蹂듭궗
				m_PlayPage = m_NextPlayPage;
				if( m_IndexPlayingPage != wNextPlayPage )
					bPlayRepeatCount = m_PlayPage.header.repeat;
				m_PageStepCount = 1;
				m_IndexPlayingPage = wNextPlayPage;
			}

			if( m_PageStepCount == m_PlayPage.header.stepnum ) // 留덉�留� �뒪�뀦�씠�씪硫�
			{
				// �떎�쓬 �럹�씠吏� 濡쒕뵫
				if( m_StopPlaying == true ) // 紐⑥뀡 �젙吏� 紐낅졊�씠 �엳�떎硫�
				{
					wNextPlayPage = m_PlayPage.header.exit; // �떎�쓬 �럹�씠吏��뒗 Exit �럹�씠吏�濡�
				}
				else
				{
					bPlayRepeatCount--;
					if( bPlayRepeatCount > 0 ) // 諛섎났 �슏�닔媛� �궓�븯�떎硫�
						wNextPlayPage = m_IndexPlayingPage; // �떎�쓬 �럹�씠吏��뒗 �쁽�옱 �럹�씠吏�濡�
					else // 諛섎났�쓣 �떎�뻽�떎硫�
						wNextPlayPage = m_PlayPage.header.next; // �떎�쓬 �럹�씠吏��뒗 Next �럹�씠吏�濡�
				}

				if( wNextPlayPage == 0 ) // �옱�깮�븷 �떎�쓬 �럹�씠吏�媛� �뾾�떎硫� �쁽�옱 �뒪�뀦源뚯��븯怨� 醫낅즺
					m_PlayingFinished = true;
				else
				{
					// �떎�쓬�럹�씠吏� 濡쒕뵫(媛숈쑝硫� 硫붾え由� 蹂듭궗, �떎瑜대㈃ �뙆�씪 �씫湲�)
					if( m_IndexPlayingPage != wNextPlayPage )
						LoadPage( wNextPlayPage, &m_NextPlayPage );
					else
						m_NextPlayPage = m_PlayPage;

					// �옱�깮�븷 �젙蹂닿� �뾾�떎硫� �쁽�옱 �뒪�뀦源뚯��븯怨� 醫낅즺
					if( m_NextPlayPage.header.repeat == 0 || m_NextPlayPage.header.stepnum == 0 )
						m_PlayingFinished = true;
				}
			}

			//////// Step �뙆�씪誘명꽣 怨꾩궛
			wPauseTime = (((unsigned short)m_PlayPage.step[m_PageStepCount-1].pause) << 5) / m_PlayPage.header.speed;
			wMaxSpeed256 = ((unsigned short)m_PlayPage.step[m_PageStepCount-1].time * (unsigned short)m_PlayPage.header.speed) >> 5;
			if( wMaxSpeed256 == 0 )
				wMaxSpeed256 = 1;
			wMaxAngle1024 = 0;

			////////// Joint蹂� �뙆�씪誘명꽣 怨꾩궛
			for( bID = 1; bID <= 20; bID++ )
			{
				// if(m_Joint.GetEnable(bID) == true)
				{
					// �씠�쟾, �쁽�옱, 誘몃옒瑜� 諛뷀깢�쑝濡� 沅ㅼ쟻�쓣 怨꾩궛
					ipAccelAngle1024[bID] = 0;

					// Find current target angle
					if( m_PlayPage.step[m_PageStepCount-1].position[bID] & INVALID_BIT_MASK )
						wCurrentTargetAngle = wpTargetAngle1024[bID];
					else
						wCurrentTargetAngle = m_PlayPage.step[m_PageStepCount-1].position[bID];

					// Update start, prev_target, curr_target
					wpStartAngle1024[bID] = wpTargetAngle1024[bID];
					wPrevTargetAngle = wpTargetAngle1024[bID];
					wpTargetAngle1024[bID] = wCurrentTargetAngle;

					// Find Moving offset
					ipMovingAngle1024[bID] = (int)(wpTargetAngle1024[bID] - wpStartAngle1024[bID]);

					// Find Next target angle
					if( m_PageStepCount == m_PlayPage.header.stepnum ) // �쁽�옱 �뒪�뀦�씠 留덉�留됱씠�씪硫�
					{
						if( m_PlayingFinished == true ) // �걹�궇 �삁�젙�씠�씪硫�
							wNextTargetAngle = wCurrentTargetAngle;
						else
						{
							if( m_NextPlayPage.step[0].position[bID] & INVALID_BIT_MASK )
								wNextTargetAngle = wCurrentTargetAngle;
							else
								wNextTargetAngle = m_NextPlayPage.step[0].position[bID];
						}
					}
					else
					{
						if( m_PlayPage.step[m_PageStepCount].position[bID] & INVALID_BIT_MASK )
							wNextTargetAngle = wCurrentTargetAngle;
						else
							wNextTargetAngle = m_PlayPage.step[m_PageStepCount].position[bID];
					}

					// Find direction change
					if( ((wPrevTargetAngle < wCurrentTargetAngle) && (wCurrentTargetAngle < wNextTargetAngle))
							|| ((wPrevTargetAngle > wCurrentTargetAngle) && (wCurrentTargetAngle > wNextTargetAngle)) )
					{
						// 怨꾩냽 利앷��븯嫄곕굹 媛먯냼�븯怨�, �샊�� 媛숇떎硫�(利�, 遺덉뿰�냽 �젏�씠 �뾾�떎硫�)
						bDirectionChanged = 0;
					}
					else
					{
						bDirectionChanged = 1;
					}

					// Find finish type
					if( bDirectionChanged || wPauseTime || m_PlayingFinished == true )
					{
						bpFinishType[bID] = ZERO_FINISH;
					}
					else
					{
						bpFinishType[bID] = NONE_ZERO_FINISH;
					}

					if( m_PlayPage.header.schedule == SPEED_BASE_SCHEDULE )
					{
						//MaxAngle1024 update
						if( ipMovingAngle1024[bID] < 0 )
							wTmp = -ipMovingAngle1024[bID];
						else
							wTmp = ipMovingAngle1024[bID];

						if( wTmp > wMaxAngle1024 )
							wMaxAngle1024 = wTmp;
					}
				}
			}

			//�떆媛꾩쓣 怨꾩궛�빐�꽌 �떎�떆 7.8msec濡� �굹�늻�떎(<<7)-洹몄떆媛꾨룞�븞�뿉 7.8msec媛� 紐뉕컻�뱾�뼱媛��뒗吏� 怨꾩궛�븳 寃�
			//�떒�쐞 蹂��솚�뮘�뿉 媛�/�냽�룄瑜� 援ы븯怨�(�떆媛�)洹� �떆媛꾩뿉 �떎�떆 7.8msec媛� 紐뉕컻�뱾�뼱媛��엳�뒗吏� 怨꾩궛
			//�떒�쐞 蹂��솚 ---  媛� :1024怨�->300�룄怨�,  �냽�룄: 256怨� ->720怨�
			//wUnitTimeNum = ((wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) /7.8msec;
			//             = ((128*wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) ;    (/7.8msec == *128)
			//             = (wMaxAngle1024*40) /(wMaxSpeed256 *3);
			if( m_PlayPage.header.schedule == TIME_BASE_SCHEDULE )
				wUnitTimeTotalNum  = wMaxSpeed256; //TIME BASE 051025
			else
				wUnitTimeTotalNum  = (wMaxAngle1024 * 40) / (wMaxSpeed256 * 3);

			wAccelStep = m_PlayPage.header.accel;
			if( wUnitTimeTotalNum <= (wAccelStep << 1) )
			{
				if( wUnitTimeTotalNum == 0 )
					wAccelStep = 0;
				else
				{
					wAccelStep = (wUnitTimeTotalNum - 1) >> 1;
					if( wAccelStep == 0 )
						wUnitTimeTotalNum = 0; //��吏곸씠�젮硫� �쟻�뼱�룄 媛��냽,�벑�냽�씠 �븳 �뒪�뀦�씠�긽�뵫�� 議댁옱�빐�빞
				}
			}

			ulTotalTime256T = ((unsigned long)wUnitTimeTotalNum) << 1;// /128 * 256
			ulPreSectionTime256T = ((unsigned long)wAccelStep) << 1;// /128 * 256
			ulMainTime256T = ulTotalTime256T - ulPreSectionTime256T;
			lDivider1 = ulPreSectionTime256T + (ulMainTime256T << 1);
			lDivider2 = (ulMainTime256T << 1);

			if(lDivider1 == 0)
				lDivider1 = 1;

			if(lDivider2 == 0)
				lDivider2 = 1;

			for( bID = 1; bID < NUMBER_OF_JOINTS; bID++ )
			{
				// if(m_Joint.GetEnable(bID) == true)
				{
					lStartSpeed1024_PreTime_256T = (long)ipLastOutSpeed1024[bID] * ulPreSectionTime256T; //  *300/1024 * 1024/720 * 256 * 2
					lMovingAngle_Speed1024Scale_256T_2T = (((long)ipMovingAngle1024[bID]) * 2560L) / 12;

					if( bpFinishType[bID] == ZERO_FINISH )
						ipMainSpeed1024[bID] = (short int)((lMovingAngle_Speed1024Scale_256T_2T - lStartSpeed1024_PreTime_256T) / lDivider2);
					else
						ipMainSpeed1024[bID] = (short int)((lMovingAngle_Speed1024Scale_256T_2T - lStartSpeed1024_PreTime_256T) / lDivider1);

					if( ipMainSpeed1024[bID] > 1023 )
						ipMainSpeed1024[bID] = 1023;

					if( ipMainSpeed1024[bID] < -1023 )
						ipMainSpeed1024[bID] = -1023;
				}
			}

			wUnitTimeNum = wAccelStep; //PreSection
		}
	}

	return false;
}

bool ActionScript::getJointName(const int &id, std::string& joint_name)
{
	/*
    // Use param server
    if(m_portNum == NotCount)
    {
        if(isJointNameInParam() == false || m_portNum == NoPort)
            return false;
    }

    for(int pi = 0; pi < m_portNum; pi++)
    {
        std::stringstream _dev_idx;
        _dev_idx << pi;

        int _dxl_num;
        if(param_nh.getParam("serial_ports/dxl_tty"+_dev_idx.str()+"/dxl_num", _dxl_num) == false || _dxl_num < 0)
            return false;

        for(int di = 0; di < _dxl_num; di++)
        {
            int _id = -1;
            joint_name = "";

            std::stringstream _dxl_idx;
            _dxl_idx << di;
            nh.getParam("serial_ports/dxl_tty"+dev_idx.str()+"/dxl"+_dxl_idx.str()+"/id", _id);

            if(_id == id)
            {
                if(nh.getParam("serial_ports/dxl_tty"+dev_idx.str()+"/dxl"+_dxl_idx.str()+"/joint_name", joint_name) == true)
                {
                    return true;
                }
                return false;
            }
        }
    }
	 */

	if(id > 0 && id < NUMBER_OF_JOINTS)
	{
		joint_name = joint_names[id];
		return true;
	}
	return false;
}

void ActionScript::motionControlCallback(const std_msgs::Int16::ConstPtr &msg)
{
	if(m_ActionFile == 0)
	{
		ROS_ERROR("Motion file is not yet opened.");
		return;
	}

	if(m_Playing == true)
	{
		ROS_ERROR("Motion is playing.");
		return;
	}

	// set flag to play motion
	Start(msg->data);
}

void ActionScript::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	// ROS_INFO("get current");
	// update current joint angle
	current_joint_state = *msg;
}

void ActionScript::Process()
{
	if(m_ActionFile == 0) return;

	// control joint thread
	comm_thread = boost::thread(boost::bind(&ActionScript::comm_thread_proc, this));
}

bool ActionScript::LoadFile( char* filename )
{
	// open action file
	FILE *action = fopen( filename, "r+b" );

	ROS_INFO("here??");
#ifdef WEBOTS
	// Olivier.Michel@cyberbotics.com added the following line to allow opening a readonly file located in the Webots installation directory.
	// This is mainly problematic on Windows
	if( action == 0 ) action = fopen( filename, "rb" );
#endif

	if( action == 0 )
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Can not open Action file!\n");
		return false;
	}

	// check it is action file
	fseek( action, 0, SEEK_END );
	if( ftell(action) != (long)(sizeof(PAGE) * MAXNUM_PAGE) )
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "It's not an Action file!\n");
		fclose( action );
		return false;
	}

	// close existed action file
	if(m_ActionFile != 0)
		fclose( m_ActionFile );

	m_ActionFile = action;
	if(DEBUG_PRINT == true)
		fprintf(stderr, "Complete loading Action file!\n");
	return true;
}

bool ActionScript::Start(int iPage)
{
	// check index of page
	if( iPage < 1 || iPage >= MAXNUM_PAGE )
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Can not play page.(%d is invalid index)\n", iPage);
		return false;
	}

	PAGE page;
	if( LoadPage(iPage, &page) == false )
		return false;

	return Start(iPage, &page);
}

bool ActionScript::Start(int index, PAGE *pPage)
{
	if(m_Playing == true)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Can not play page %d.(Now playing)\n", index);
		return false;
	}

	m_PlayPage = *pPage;

	if( m_PlayPage.header.repeat == 0 || m_PlayPage.header.stepnum == 0 )
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Page %d has no action\n", index);
		return false;
	}

	m_IndexPlayingPage = index;
	m_FirstDrivingStart = true;
	m_Playing = true;
	return true;
}

void ActionScript::Stop()
{
	m_StopPlaying = true;
}

bool ActionScript::LoadPage(int index, PAGE *pPage)
{
	long position = (long)(sizeof(PAGE)*index);

	if( fseek( m_ActionFile, position, SEEK_SET ) != 0 )
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Page %d is not found.\n", index);
		return false;
	}

	if( fread( pPage, 1, sizeof(PAGE), m_ActionFile ) != sizeof(PAGE) )
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Page %d is not read.\n", index);
		return false;
	}

	if( VerifyChecksum( pPage ) == false )
		ResetPage( pPage );

	return true;
}

bool ActionScript::VerifyChecksum( PAGE *pPage )
{
	unsigned char checksum = 0x00;
	unsigned char *pt = (unsigned char*)pPage;

	for(unsigned int i = 0; i < sizeof(PAGE); i++)
	{
		checksum += *pt;
		pt++;
	}

	if(checksum != 0xff)
		return false;

	return true;
}

void ActionScript::SetChecksum( PAGE *pPage )
{
	unsigned char checksum = 0x00;
	unsigned char *pt = (unsigned char*)pPage;

	pPage->header.checksum = 0x00;

	for(unsigned int i=0; i<sizeof(PAGE); i++)
	{
		checksum += *pt;
		pt++;
	}

	pPage->header.checksum = (unsigned char)(0xff - checksum);
}

void ActionScript::ResetPage(PAGE *pPage)
{
	/*
    unsigned char *pt = (unsigned char*)pPage;

    for(unsigned int i=0; i<sizeof(PAGE); i++)
    {
	 *pt = 0x00;
        pt++;
    }

    pPage->header.schedule = TIME_BASE_SCHEDULE; // default time base
    pPage->header.repeat = 1;
    pPage->header.speed = 32;
    pPage->header.accel = 32;

    for(int i=0; i<JointData::NUMBER_OF_JOINTS; i++)
        pPage->header.slope[i] = 0x55;

    for(int i=0; i<MAXNUM_STEP; i++)
    {
        for(int j=0; j<31; j++)
            pPage->step[i].position[j] = INVALID_BIT_MASK;

        pPage->step[i].pause = 0;
        pPage->step[i].time = 0;
    }

    SetChecksum( pPage );
	 */
}

}
