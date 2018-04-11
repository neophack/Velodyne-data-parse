/*
* How to read a packet capture file.
*/

/*
* Step 1 - Add includes
*/
#include <string>
#include <iostream>
#include <pcap.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "C:\Program Files (x86)\Microsoft SDKs\Windows\v7.1A\Include\WinSock2.h"
#include "tinystr.h"
#include "tinyxml.h"
#include "hdl64parse.h"

using namespace std;

#pragma comment (lib, "wpcap.lib") 

#define UNIT_TRANSFORM 0.2
#define HDL_NUM_ROT_ANGLES 36000
#define HDL64_FIRE_NUM 64
#define HDL_PACKAGE_FRAME  34700
#define DIS_LSB 0.002
#define ONEDEGREE_2_RAD 0.0174533
#define ENET_H_LEN 42

namespace TABLE
{
	std::vector<double> cos_vertc;
	std::vector<double> sin_vertc;
	std::vector<double> sin_rot;
	std::vector<double> cos_rot;
	std::vector<double> sin_rotc;
	std::vector<double> cos_rotc;
}

struct laserLine
{
	int id;
	double rotCorrection;
	double vertCorrection;
	double distCorrection;
	double distCorrectionX;
	double distCorrectionY;
	double vertOffsetCorrection;
	double horizOffsetCorrection;
	double focalDistance;
	double focalSlope;
};
struct MaxIntensity
{
	int maxintensity_;
};
struct MinIntensity
{
	int minintensity_;
};

struct Fire
{
	double x_;
	double y_;
	double z_;
	double range_;
	int intensity_;
	unsigned char block_id;
	int ring_;    //id
	int spin_;
	double timestamp;
};


vector<laserLine> correction;
vector<MaxIntensity> maxintensity_set;
vector<MinIntensity> minintensity_set;

int correctionLoad()
{
	const char* filepath = "db.xml";
	TiXmlDocument doc(filepath);
	bool loadOkay = doc.LoadFile();
	if (!loadOkay) {
		printf("Could not load test file %s. Error='%s'. Exiting.\n", filepath, doc.ErrorDesc());
		exit(1);
	}

	TiXmlElement* root = doc.RootElement();
	TiXmlNode* points = root->FirstChild("DB")->FirstChild("points_");
	TiXmlNode* minIntensity = root->FirstChild("DB")->FirstChild("minIntensity_");
	TiXmlNode* maxIntensity = root->FirstChild("DB")->FirstChild("maxIntensity_");
	int corretioncount = 0;
	for (TiXmlNode* item = points->FirstChild("item"); item; item = item->NextSibling("item"))
	{
		laserLine temp;
		temp.id = atoi(item->FirstChild("px")->FirstChild("id_")->ToElement()->GetText());
		temp.rotCorrection = atof(item->FirstChild("px")->FirstChild("rotCorrection_")->ToElement()->GetText());
		temp.rotCorrection *= ONEDEGREE_2_RAD;
		temp.vertCorrection = atof(item->FirstChild("px")->FirstChild("vertCorrection_")->ToElement()->GetText());
		temp.vertCorrection *= ONEDEGREE_2_RAD;
		temp.distCorrection = atof(item->FirstChild()->FirstChild("distCorrection_")->ToElement()->GetText()) / 100;
		temp.distCorrectionX = atof(item->FirstChild("px")->FirstChild("distCorrectionX_")->ToElement()->GetText()) / 100;
		temp.distCorrectionY = atof(item->FirstChild("px")->FirstChild("distCorrectionY_")->ToElement()->GetText()) / 100;
		temp.vertOffsetCorrection = atof(item->FirstChild("px")->FirstChild("vertOffsetCorrection_")->ToElement()->GetText()) / 100;
		temp.horizOffsetCorrection = atof(item->FirstChild("px")->FirstChild("horizOffsetCorrection_")->ToElement()->GetText()) / 100;
		temp.focalDistance = atof(item->FirstChild("px")->FirstChild("focalDistance_")->ToElement()->GetText()) / 100;
		temp.focalSlope = atof(item->FirstChild("px")->FirstChild("focalSlope_")->ToElement()->GetText()) * 0.5;
		corretioncount++;
		correction.push_back(temp);
	}
	int laser = 0;
	for (TiXmlNode* item = maxIntensity->FirstChild("item"); item ; item = item->NextSibling("item"))
	{
		MaxIntensity intentemp;
		intentemp.maxintensity_ = atoi(item->ToElement()->GetText());
		maxintensity_set.push_back(intentemp);
	}
	for (TiXmlNode* item = minIntensity->FirstChild("item"); item ; item = item->NextSibling("item"))
	{
		MinIntensity intentemp;
		intentemp.minintensity_ = atoi(item->ToElement()->GetText());
		minintensity_set.push_back(intentemp);
	}

}


void HDLTableInit()
{
	int beam;
	int angle;
	//ONEDEGREE_2_RAD = 0.0174533
	for (beam = 0; beam < HDL64_FIRE_NUM ; beam++)
	{
		TABLE::cos_rotc.push_back(cos(correction[beam].rotCorrection ));
		TABLE::cos_vertc.push_back(cos(correction[beam].vertCorrection ));
		TABLE::sin_rotc.push_back(sin(correction[beam].rotCorrection ));
		TABLE::sin_vertc.push_back(sin(correction[beam].vertCorrection ));
	}
	//HDL_NUM_ROT_ANGLES 36000
	for (angle = 0; angle < HDL_NUM_ROT_ANGLES; angle++)
	{
		TABLE::cos_rot.push_back(cos(angle * 0.01 * ONEDEGREE_2_RAD));
		TABLE::sin_rot.push_back(sin(angle * 0.01 * ONEDEGREE_2_RAD));
	}
}

void recorderOut(ofstream &file, vector<Fire> loop_point)
{
	for (auto line : loop_point)
	{
		file << line.x_ << " " << line.y_ << " " << line.ring_ << " " << line.spin_ << " " << fixed << setprecision(6) << line.timestamp << endl;
		//file << fixed << setprecision(6) << line.timestamp << " " << line.value << " " << line.type << endl;
	}
	file.close();
}

int recorder_count = 0;

vector<Fire> loop_point;
bool firstFlag = true;
int loop_count = 0;

int main(int argc, char *argv[])
{
	/*
	* Step 2 - Get a file name
	*/

	string file = "12049(Frame13to16).pcap"; 
	//string file = "xiepo2.pcap";
	//string file = "2017-4-13电信马桶.pcap";
	//string file = "20170725velofile01.pcap";
	//string file = "1120_HDL64.pcap";
	//string file = "2017-07-25_06_38_39_vlf04.pcap";
	//string file = "2017-07-25_06_44_53_vlf05.pcap";

	/*
	* Step 3 - Create an char array to hold the error.
	*/

	// Note: errbuf in pcap_open functions is assumed to be able to hold at least PCAP_ERRBUF_SIZE chars
	//       PCAP_ERRBUF_SIZE is defined as 256.
	// http://www.winpcap.org/docs/docs_40_2/html/group__wpcap__def.html
	char errbuff[PCAP_ERRBUF_SIZE];

	/*
	* Step 4 - Open the file and store result in pointer to pcap_t
	*/

	// Use pcap_open_offline
	// http://www.winpcap.org/docs/docs_41b5/html/group__wpcapfunc.html#g91078168a13de8848df2b7b83d1f5b69
	pcap_t * pcap = pcap_open_offline(file.c_str(), errbuff);

	/*
	* Step 5 - Create a header and a data object
	*/

	// Create a header object:
	// http://www.winpcap.org/docs/docs_40_2/html/structpcap__pkthdr.html
	struct pcap_pkthdr *header;

	// Create a character array using a u_char
	// u_char is defined here:
	// C:\Program Files (x86)\Microsoft SDKs\Windows\v7.0A\Include\WinSock2.h
	// typedef unsigned char   u_char;
	const u_char *data;

	/*
	* Step 6 - Loop through packets and print them to screen
	*/
	u_int packetCount = 0;

	u_int radionum = 0;
	correctionLoad();
	
	HDLTableInit();
	cout << pcap_next_ex(pcap, &header, &data) << endl;
	while (int returnValue = pcap_next_ex(pcap, &header, &data) >= 0)//处理一个包的函数
	{
		// Print using printf. See printf reference:
		// http://www.cplusplus.com/reference/clibrary/cstdio/printf/
		
		
		// Show the packet number
		//printf("Packet # %i\n", ++packetCount);

		// Show the size in bytes of the packet
		//printf("Packet size: %d bytes\n", header->len);


/**************************************************************************************************************************************/
		//char type = data[ENET_H_LEN + 1204];

		//int value = data[ENET_H_LEN + 1205];
		//printf("value = %x ", value); // %x 表示以十六进制输出
		//cout << " type: " << type << endl;


		//evrty packet has has one timestamp
		double total_us = data[ENET_H_LEN + 1203] * pow(2, 24) + data[ENET_H_LEN + 1202] * pow(2, 16) + data[ENET_H_LEN + 1201] * pow(2, 8) + data[ENET_H_LEN + 1200] * pow(2, 0);
		double timestamp1 = (total_us) / pow(10, 6);

		for (size_t j = 0; j < 12; j++)//evrty packet has 12 block,has 12 spin_angle
		{
			//cout << "this frame timestamp(s): " << fixed << setprecision(6) << timestamp1 << endl;
			unsigned char block_id;
			int block = data[ENET_H_LEN + 1 + j * 100] * 16 * 16 + data[ENET_H_LEN + j * 100];

			if (block == 0xEEFF)
				block_id = '1';  //upper 32 laser
			else if (block == 0xDDFF)
				block_id = '0';	 //lower 32 laser

			spin_now = data[ENET_H_LEN + 3 + j * 100] * 16 * 16 + data[ENET_H_LEN + 2 + j * 100]; //ÕâÀïºÃÏñÊÇÔÚÈ¡16½øÖÆµÄÒ»¸öÊý£ºlaser BlockID »òÕßÊÇ Rotational Position

			//spincheck << spin_now << " " << spin_pre <<" "<< spin_now - spin_pre << endl;

			if ((spin_now < 18000 || spin_now > 18010) && firstFlag)//start angle 18009开始记录
				continue;
			else
				firstFlag = false;
		
			int distancelocal = 0;

			if (j % 2 == 0) {

				beam_n = 0;

				for (int i = 4; i < 99; i += 3) {//every block has 32 points
					//****************************coordinate calibration begin************************************************
					dis = (unsigned char)data[ENET_H_LEN + i + j * 100 + 1] * 16 * 16 +
						(unsigned char)data[ENET_H_LEN + i + j * 100];//5209
					intensity = (unsigned char)data[ENET_H_LEN + i + j * 100 + 2];//139

					int intensity_raw = intensity;
					int intensity_dif = 0;

					COS_VC_n = TABLE::cos_vertc[beam_n];//0.9999975
					SIN_VC_n = TABLE::sin_vertc[beam_n];//-0.00222405
					SIN_ROT_n = TABLE::sin_rot[spin_now];//

					COS_ROT_n = TABLE::cos_rot[spin_now];//

					COS_RC_n = TABLE::cos_rotc[beam_n];//0.9999989
					SIN_RC_n = TABLE::sin_rotc[beam_n];//-0.001435

					disC_n = correction[beam_n].distCorrection;//1.31835
					disCX_n = correction[beam_n].distCorrectionX;//1.34808
					disCY_n = correction[beam_n].distCorrectionY;//1.31460
					vertoffC_n = correction[beam_n].vertOffsetCorrection;//0.21578
					horioffC_n = correction[beam_n].horizOffsetCorrection;//0.026
					rotc = correction[beam_n].rotCorrection;//-0.08223

					distancetemp = (dis * DIS_LSB);//10.406
					::distance = distancetemp + disC_n;//11.72

					cosRotAngle = ((COS_ROT_n)* (COS_RC_n)+(SIN_ROT_n)* (SIN_RC_n));//0.9999989
					sinRotAngle = ((SIN_ROT_n)* (COS_RC_n)-(COS_ROT_n)* (SIN_RC_n));//0.001435

					disxy = (COS_VC_n)* ::distance - vertoffC_n * SIN_VC_n;//modi ROS	11.65671

					xx = abs((disxy * sinRotAngle - horioffC_n * cosRotAngle));//0.9315
					yy = abs((disxy * cosRotAngle + horioffC_n * sinRotAngle)); //add ROS	11.6194

																				//distanceCorrX = ((disC_n - disCX_n) * (xx - 240) / (2504 - 240) + disCX_n); //
					distanceCorrX = ((disC_n - disCX_n) * (xx - 2.4) / (25.04 - 2.4)) + disCX_n; //modi ROS
					distanceCorrX -= disC_n;  //add ROS	0.03

											  //distanceCorrY = ((disC_n - disCY_n) * (yy - 0) / (2504 - 0) + disCY_n); //
					distanceCorrY = ((disC_n - disCY_n) * (yy - 1.93) / (25.04 - 1.93) + disCY_n); //modi ROS
					distanceCorrY -= disC_n;  //add ROS

					distanceX = ::distance + distanceCorrX;//11.756
														   //disxy = ((COS_VC_n)* distanceX);  //
					disxy = (COS_VC_n)* distanceX - vertoffC_n * SIN_VC_n;  //modi ROS	11.688

					x = (disxy * sinRotAngle - horioffC_n * cosRotAngle);//0.934

					distanceY = ::distance + distanceCorrY;//
					disxy = ((COS_VC_n)* distanceY) - vertoffC_n * SIN_VC_n;  //modi ROS	11.65455
					y = (disxy * cosRotAngle + horioffC_n * sinRotAngle);  //	11.6173
					z = (distanceY * SIN_VC_n + vertoffC_n * COS_VC_n); //modi rotationstart0.18968

					//****************************coordinate calibration done************************************************

					//****************************intensity calibration begin************************************************

					int min_intensity = 0, max_intensity = 255;
					float focaldistance = correction[beam_n].focalDistance;	//24
					float focalslope = correction[beam_n].focalSlope;//0.5
					float focaloffset = 256 * (1 - focaldistance / 13100)* (1 - focaldistance / 13100);//255.06

					intensity += focalslope*(abs(focaloffset - 256 * (1 - (dis * 1.0 / 65535))*(1 - (dis * 1.0 / 65535))));  //ÕâÀïµÄdisÊÇÄÄ¸ö¾àÀëÓÐ´ý¿¼¾¿

					if (intensity < min_intensity) intensity = min_intensity;
					if (intensity > max_intensity) intensity = max_intensity;

					intensity_dif = intensity - intensity_raw;

					loop_point.push_back(Fire{ x, y, z, (dis*0.2),intensity, block_id, beam_n, spin_now,timestamp1 });

					//****************************intensity calibration done************************************************
					recorder_count++;
					beam_n++;
				}
			}
			else
			{
				beam_n = 0;

				for (int i = 4; i < 99; i += 3)
				{
					//****************************coordinate calibration begin************************************************

					dis = (unsigned char)data[ENET_H_LEN + i + j * 100 + 1] * 16 * 16 +
						(unsigned char)data[ENET_H_LEN + i + j * 100];
					intensity = (unsigned char)data[ENET_H_LEN + i + j * 100 + 2];

					int intensity_raw = intensity;
					int intensity_dif = 0;

					COS_VC_n = TABLE::cos_vertc[beam_n + 32];//
					SIN_VC_n = TABLE::sin_vertc[beam_n + 32];//
					SIN_ROT_n = TABLE::sin_rot[spin_now];//

					COS_ROT_n = TABLE::cos_rot[spin_now];//

					COS_RC_n = TABLE::cos_rotc[beam_n + 32];//
					SIN_RC_n = TABLE::sin_rotc[beam_n + 32];//

					disC_n = correction[beam_n + 32].distCorrection;//
					disCX_n = correction[beam_n + 32].distCorrectionX;//
					disCY_n = correction[beam_n + 32].distCorrectionY;//
					vertoffC_n = correction[beam_n + 32].vertOffsetCorrection;//
					horioffC_n = correction[beam_n + 32].horizOffsetCorrection;//
					rotc = correction[beam_n + 32].rotCorrection;//

					distancetemp = (dis * DIS_LSB);//
					::distance = distancetemp + disC_n;//

					cosRotAngle = ((COS_ROT_n)* (COS_RC_n)+(SIN_ROT_n)* (SIN_RC_n));//
					sinRotAngle = ((SIN_ROT_n)* (COS_RC_n)-(COS_ROT_n)* (SIN_RC_n));//

					disxy = (COS_VC_n)* ::distance - vertoffC_n * SIN_VC_n;//modi ROS

					xx = abs((disxy * sinRotAngle - horioffC_n * cosRotAngle));//
					yy = abs((disxy * cosRotAngle + horioffC_n * sinRotAngle)); //add ROS

					distanceCorrX = ((disC_n - disCX_n) * (xx - 2.4) / (25.04 - 2.4)) + disCX_n; //modi ROS
					distanceCorrX -= disC_n;  //add ROS

					distanceCorrY = ((disC_n - disCY_n) * (yy - 1.93) / (25.04 - 1.93) + disCY_n); //modi ROS
					distanceCorrY -= disC_n;  //add ROS

					distanceX = ::distance + distanceCorrX;//

					disxy = (COS_VC_n)* distanceX - vertoffC_n * SIN_VC_n;  //modi ROS

					x = (disxy * sinRotAngle - horioffC_n * cosRotAngle);//

					distanceY = ::distance + distanceCorrY;//
					disxy = ((COS_VC_n)* distanceY) - vertoffC_n * SIN_VC_n;  //modi ROS
					y = (disxy * cosRotAngle + horioffC_n * sinRotAngle);  //
					z = (distanceY * SIN_VC_n + vertoffC_n * COS_VC_n); //modi ROS


					//****************************coordinate calibration done************************************************

					//****************************intensity calibration************************************************

					int min_intensity = 0, max_intensity = 255;
					float focaldistance = correction[beam_n + 32].focalDistance;
					float focalslope = correction[beam_n + 32].focalSlope;
					float focaloffset = 256 * (1 - focaldistance / 13100)* (1 - focaldistance / 13100);

					intensity += focalslope*(abs(focaloffset - 256 * (1 - (dis * 1.0 / 65535))*(1 - (dis * 1.0 / 65535))));  //ÕâÀïµÄdisÊÇÄÄ¸ö¾àÀëÓÐ´ý¿¼¾¿

					if (intensity < min_intensity) intensity = min_intensity;
					if (intensity > max_intensity) intensity = max_intensity;

					intensity_dif = intensity - intensity_raw;

					loop_point.push_back(Fire{ x, y, z, (dis*0.2),intensity, block_id, beam_n, spin_now,timestamp1 });

					//****************************intensity calibration done************************************************
					recorder_count++;
					beam_n++;
				}
			}
			if (radionum > 300 && spin_now > 18000 && spin_now < 18020)
			{
				stringstream stream_buff;
				stream_buff << loop_count;
				ofstream loopfile("..//HDLdata//" + stream_buff.str() + ".txt");
				recorderOut(loopfile, loop_point);
				cout << "record " << stream_buff.str() << " success!" << endl;
				loop_point.clear();
				loop_count++;
				radionum = 0;
			}
		}//for(12)
		radionum++;//one packet finish
	}//while
}

