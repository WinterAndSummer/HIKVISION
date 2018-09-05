
/**	@file       mian.cpp
*	@note       Hikvision Digital Technology Co., Ltd. All Right Reserved.
*	@brief
*
*	@author     lipengfei
*	@date       2018/05/10
*	@note       ��ʷ��¼��
*	@note       V1.0.0
*	@warning
*/


#include <stdlib.h>
#include <stdio.h>
#include "OSSocket.h"
#include "JsonParse.h"
#include "CmdParse.h"
#include <iostream>
#include "AStar.hpp"
#include <vector>
#include <cmath>
#include<time.h>
#include <stdlib.h>
#include <algorithm>


#define MAX_SOCKET_BUFFER       (1024 * 1024 * 4)       /// ���ͽ����������4M



/** @fn     int RecvJuderData(OS_SOCKET hSocket, char *pBuffer)
*  @brief	��������
*	@param  -I   - OS_SOCKET hSocket
*	@param  -I   - char * pBuffer
*	@return int
*/



AStar::Generator generator;//map
REGION regionTable[16];
//std::vector<ATTACK> Enemygoods;
UAV_PRICE  cheapestUav;
UAV_PRICE  transportUav;
int enemyParkingX = 0;
int enemyParkingY = 0;
std::vector<std::vector<DIS_INFO>> disInfo;


int RecvJuderData(OS_SOCKET hSocket, char *pBuffer)
{
	int         nRecvLen = 0;
	int         nLen = 0;

	while (1)
	{
		// ����ͷ������
		nLen = OSRecv(hSocket, pBuffer + nRecvLen, MAX_SOCKET_BUFFER);
		if (nLen <= 0)
		{
			printf("recv error\n");
			return nLen;
		}

		nRecvLen += nLen;

		if (nRecvLen >= SOCKET_HEAD_LEN)
		{
			break;
		}
	}

	int         nJsonLen = 0;
	char        szLen[10] = { 0 };

	memcpy(szLen, pBuffer, SOCKET_HEAD_LEN);

	nJsonLen = atoi(szLen);

	while (nRecvLen < (SOCKET_HEAD_LEN + nJsonLen))
	{
		// ˵�����ݻ�û������
		nLen = OSRecv(hSocket, pBuffer + nRecvLen, MAX_SOCKET_BUFFER);
		if (nLen <= 0)
		{
			printf("recv error\n");
			return nLen;
		}

		nRecvLen += nLen;
	}

	return 0;
}

/** @fn     int SendJuderData(OS_SOCKET hSocket, char *pBuffer, int nLen)
*  @brief	��������
*	@param  -I   - OS_SOCKET hSocket
*	@param  -I   - char * pBuffer
*	@param  -I   - int nLen
*	@return int
*/
int SendJuderData(OS_SOCKET hSocket, char *pBuffer, int nLen)
{
	int     nSendLen = 0;
	int     nLenTmp = 0;

	while (nSendLen < nLen)
	{
		nLenTmp = OSSend(hSocket, pBuffer + nSendLen, nLen - nSendLen);
		if (nLenTmp < 0)
		{
			return -1;
		}

		nSendLen += nLenTmp;
	}

	return 0;
}
//�����Ƿ���ײ
int IsCollision(AStar::Vec3i uav1Start, AStar::Vec3i uav1End, AStar::Vec3i uav2Start, AStar::Vec3i uav2End)
{
	//���������е�����
	float mid1x = uav1Start.x + uav1End.x;
	float mid1y = uav1Start.y + uav1End.y;
	float mid1z = uav1Start.z + uav1End.z;

	float mid2x = uav2Start.x + uav2End.x;
	float mid2y = uav2Start.y + uav2End.y;
	float mid2z = uav2Start.z + uav2End.z;
	if (uav1End == uav2End)
		return 1;//��ͬһ������;
	else if (uav1End == uav2Start&&uav1Start == uav2End)
		return 2;//����
	else if (mid1x == mid2x&&mid1y == mid2y&&mid1z == mid2z)
		return 3;
	else
		return 0;//����ײ
}

void SetMap(MAP_INFO *pstMap, MATCH_STATUS * pstMatch)
{
	generator.setWorldSize({ pstMap->nMapX, pstMap->nMapY, pstMap->nMapZ });
	for (int n_buiding = 0; n_buiding<pstMap->nBuildingNum; n_buiding++)
	{
		int buidingX = pstMap->astBuilding[n_buiding].nX;
		int buidingY = pstMap->astBuilding[n_buiding].nY;
		int buidingW = pstMap->astBuilding[n_buiding].nW;
		int buidingL = pstMap->astBuilding[n_buiding].nL;
		int buidingH = pstMap->astBuilding[n_buiding].nH;
		for (int i = buidingX; i <= buidingX + buidingL - 1; i++)
			for (int j = buidingY; j <= buidingW + buidingY - 1; j++)
				for (int k = 0; k <= buidingH - 1; k++)
				{
					generator.addCollision({ i, j, k });
				}
	}
	generator.setH_Min_H_Max(pstMap->nHLow, pstMap->nHHigh);
	enemyParkingX = pstMatch->astEnemyUav[0].nX;
	enemyParkingY = pstMatch->astEnemyUav[0].nY;
}

void InitUavInfo(MAP_INFO *pstMap)
{
	int theMinPrice = 100000;
	int theMaxPrice = 0;
	UAV_PRICE expensiveUav;
	float Practicability = 0;//ʵ����
	for (int i = 0; i < pstMap->nUavPriceNum; i++)
	{
		if (pstMap->astUavPrice[i].nValue < theMinPrice)
		{
			theMinPrice = pstMap->astUavPrice[i].nValue;
			cheapestUav = pstMap->astUavPrice[i];
		}
		if (pstMap->astUavPrice[i].nValue > theMaxPrice)
		{
			theMaxPrice = pstMap->astUavPrice[i].nValue;
			expensiveUav = pstMap->astUavPrice[i];
		}
	}
	for (int i = 0; i < pstMap->nUavPriceNum; i++)
	{
		if (pstMap->astUavPrice[i].szType[1] == expensiveUav.szType[1])
			continue;
		float thePra = (float)pstMap->astUavPrice[i].nLoadWeight / pstMap->astUavPrice[i].nValue;
		if (thePra>Practicability)
		{
			Practicability = thePra;
			transportUav = pstMap->astUavPrice[i];
		}
	}


}


//�������������������·��
void  MinStepsCal(AStar::Vec3i startPoint, AStar::Vec3i endPoint, AStar::CoordinateList *bestPath)
{
	std::vector < AStar::CoordinateList > pathList;//���治ͬ���·��

	generator.setHeuristic(AStar::Heuristic::manhattan);
	generator.setDiagonalMovement(true);
	//std::cout << "Generate path ... \n";

	auto path = generator.findPath({ startPoint.x, startPoint.y, startPoint.z }, { endPoint.x, endPoint.y, endPoint.z });
	pathList.push_back(path);

	int minSteps = 0;//�˶�����С����
	int TheMinStepsNum = 0;
	for (int pathNum = 0; pathNum < pathList.size(); pathNum++)
	{
		if (pathList[pathNum].size()  > minSteps)
		{
			minSteps = pathList[pathNum].size();
			TheMinStepsNum = pathNum;
		}
	}
	*bestPath = pathList[TheMinStepsNum];

	//�����ǰѼ���õ�����ά·�����뵽pathList��
	//��һ����ֱ�ƶ����֣����������������½�

}
//��һ����z=0���ƶ�����һ����z=0��
void move2point(MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, AStar::CoordinateList MbestPath, UAV *uav)
{
	AStar::Vec3i startPoint = { uav->nX, uav->nY, uav->nZ };
	for (int i = 1; i < MbestPath.size(); i++)
	{
		if (uav->nX == MbestPath[i].x&&uav->nY == MbestPath[i].y&&uav->nZ == MbestPath[i].z)
		{
			AStar::Vec3i endPoint;
			endPoint = { MbestPath[i - 1].x, MbestPath[i - 1].y, MbestPath[i - 1].z };
			int CollisionType2 = 0;
			for (int j = 0; j < pstMatch->nUavWeNum; j++)
			{
				AStar::Vec3i jUavStartPoint = { pstMatch->astWeUav[j].nX, pstMatch->astWeUav[j].nY, pstMatch->astWeUav[j].nZ };
				AStar::Vec3i jUavEndPoint = { pstFlayPlane->astUav[j].nX, pstFlayPlane->astUav[j].nY, pstFlayPlane->astUav[j].nZ };
				if (pstFlayPlane->astUav[j].haveDeal == false)//û��������
					continue;
				else
				{
					if (1 == IsCollision(jUavStartPoint, jUavEndPoint, startPoint, endPoint))//�����ײ1
					{
					
						/*AStar::CoordinateList *newPath1 = new AStar::CoordinateList;
						generator.addCollision(jUavEndPoint);
						MinStepsCal(startPoint, MbestPath[0], newPath1);
						move2point(pstMatch, pstFlayPlane, *newPath1, uav);
						delete newPath1;
						CollisionType2 = 1;
						generator.removeCollision(jUavEndPoint);*/
						CollisionType2 = 1;
						uav->haveDeal = true;
						break;

					}
					else if (2 == IsCollision(jUavStartPoint, jUavEndPoint, startPoint, endPoint))
					{
					/*	if (i == 1)
						{
							AStar::CoordinateList *newPath2 = new AStar::CoordinateList;
							generator.addCollision(jUavStartPoint);
							AStar::Vec3i newpoint={ 0, 0, 0 };
							MinStepsCal(startPoint, newpoint, newPath2);
							move2point(pstMatch, pstFlayPlane, *newPath2, uav);
							delete newPath2;
							CollisionType2 = 2;
							generator.removeCollision(jUavStartPoint);
							break;
						}*/
							
						AStar::CoordinateList *newPath2 = new AStar::CoordinateList;
						generator.addCollision(jUavStartPoint);
						MinStepsCal(startPoint, MbestPath[0], newPath2);
						move2point(pstMatch, pstFlayPlane, *newPath2, uav);
						delete newPath2;
						CollisionType2 = 2;
						generator.removeCollision(jUavStartPoint);
						break;
					}
					else if (3 == IsCollision(jUavStartPoint, jUavEndPoint, startPoint, endPoint))
					{
						/*if (i == 1)
						{
							CollisionType2 = 3;
							break;
						}*/
						AStar::CoordinateList *newPath3 = new AStar::CoordinateList;
						generator.addCollision(jUavEndPoint);
						generator.addCollision(endPoint);
						MinStepsCal(startPoint, MbestPath[0], newPath3);
						move2point(pstMatch, pstFlayPlane, *newPath3, uav);
						delete newPath3;
						CollisionType2 = 3;
						generator.removeCollision(jUavEndPoint);
						generator.removeCollision(endPoint);
						break;
					}
				}

			}
			if (CollisionType2 == 0 && i>0)
			{
				uav->nX = MbestPath[i - 1].x;
				uav->nY = MbestPath[i - 1].y;
				uav->nZ = MbestPath[i - 1].z;
				uav->haveDeal = true;
			}
		}
	}


}

//�ͻ�����
void GetGood(MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, GOODS *good, UAV *uav, AStar::CoordinateList GbestPath1, AStar::CoordinateList GbestPath2)
{
	//��һ�׶Σ�����ȡ���ص�
	if ((uav->nX != good->nStartX || uav->nY != good->nStartY || uav->nZ != 0) && uav->nGoodsNo == -1)
	{
		move2point(pstMatch, pstFlayPlane, GbestPath1, uav);
	}
	else if (uav->nX == good->nStartX&&uav->nY == good->nStartY&&uav->nZ == 0 && uav->nGoodsNo == -1)
	{
		bool  haveGood = false;
		for (int i = 0; i < pstMatch->nGoodsNum; i++)
		{
			if (pstMatch->astGoods[i].nNO == good->nNO)
			{
				haveGood = true;
				break;
			}
		}
		if (haveGood == false)
		{
			uav->task = 0;
		}
		if (haveGood==true&&good->nState == 0)
		{
			uav->nGoodsNo = good->nNO;
			uav->path = GbestPath2;
			uav->task = 2;
			int weight = 0;
			for (int j = 0; j < pstMatch->nGoodsNum; j++)
			{
				if (pstMatch->astGoods[j].nNO == uav->nGoodsNo)
				{
					weight = pstMatch->astGoods[j].nWeight;
					break;
				}
			}
			uav->remain_electricity -= weight;
		}
	}
}

void SendGood(MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, UAV *uav, AStar::CoordinateList SbestPath )
{
	AStar::Vec3i uavPoint = { uav->nX, uav->nY, uav->nZ };
	bool havefund = false;
	for (int i = 0; i < SbestPath.size(); i++)
	{
		if (SbestPath[i] == uavPoint)
		{
			havefund = true;
			break;
		}
	}
	if (havefund == false)
	{
		AStar::CoordinateList *newPath = new AStar::CoordinateList;
		MinStepsCal(uavPoint, SbestPath[0], newPath);
		move2point(pstMatch, pstFlayPlane, *newPath, uav);
		uav->path = *newPath;
		delete newPath;
	
	}

	if ((uav->nX != SbestPath[0].x || uav->nY != SbestPath[0].y || uav->nZ != 0) && uav->nGoodsNo >= 0)
		move2point(pstMatch, pstFlayPlane, SbestPath, uav);

	if (uav->nX == SbestPath[0].x&&uav->nY == SbestPath[0].y&&uav->nZ == 0)
	{
		uav->task = 0;
	}
}

void InitRegionTable(MAP_INFO *pstMap, MATCH_STATUS * pstMatch)
{
	int spaceX = pstMap->nMapX / 4;
	int spaceY = pstMap->nMapY / 4;
	for (int i_X = 0; i_X <4; i_X++)
		for (int i_Y = 0; i_Y <4; i_Y++)
		{
		REGION temp;
		temp.minX = 0 + i_X* spaceX;
		temp.maxX = (i_X + 1)*spaceX - 1;
		temp.minY = 0 + i_Y* spaceY;
		temp.maxY = (i_Y + 1)*spaceY - 1;
		temp.midPointX = (temp.minX + temp.maxX) / 2;
		temp.midPointY = (temp.minY + temp.maxY) / 2;
		int PointX = (temp.minX + temp.maxX) / 2;
		int PointY = (temp.minY + temp.maxY) / 2;
		bool index = false;
		for (int i = temp.minX; i < temp.maxX; i++)
		{
			for (int j = temp.minX; j < temp.maxY; j++)
			{
				if (generator.map[PointX][PointY][pstMap->nHHigh] != true)
				{
					temp.midPointX = PointX;
					temp.midPointY = PointY;
					index = true;
					break;
				}
			}
			if (index == true)
				break;
		}

		int goodsnum = 0;
		for (int i = 0; i < pstMatch->nGoodsNum; i++)
		{
			if (pstMatch->astGoods[i].nStartX >= temp.minX&&pstMatch->astGoods[i].nStartX <= temp.maxX&&
				pstMatch->astGoods[i].nStartY >= temp.minY&&pstMatch->astGoods[i].nStartY <= temp.maxY)
				goodsnum++;
		}
		temp.goodsNum = goodsnum;
		temp.WePakDistance = sqrtf((pstMap->nParkingX - temp.midPointX)*(pstMap->nParkingX - temp.midPointX) + (pstMap->nParkingY - temp.midPointY)*(pstMap->nParkingY - temp.midPointX));
		temp.EnePakDistance = sqrtf((pstMatch->astEnemyUav[0].nX - temp.midPointX)*(pstMatch->astEnemyUav[0].nX - temp.midPointX) + (pstMatch->astEnemyUav[0].nY - temp.midPointY)*(pstMatch->astEnemyUav[0].nY - temp.midPointY));
		regionTable[i_X * 3 + i_Y] = temp;
		}

}
void MaintainRegionTable(MATCH_STATUS * pstMatch)
{

	for (int i = 0; i <16; i++)
	{
		int goodsnum = 0;
		for (int j = 0; j < pstMatch->nGoodsNum; j++)
		{
			if (pstMatch->astGoods[j].nStartX >= regionTable[i].minX&&pstMatch->astGoods[j].nStartX <= regionTable[i].maxX&&
				pstMatch->astGoods[j].nStartY >= regionTable[i].minY&&pstMatch->astGoods[j].nStartY <= regionTable[i].maxY)
				goodsnum++;
		}
		regionTable[i].goodsNum = goodsnum;

	}

}
AStar::CoordinateList direction(UAV uav, MAP_INFO *pstMap)
{
	AStar::CoordinateList detetDirection;
	AStar::CoordinateList direction1, direction2, direction3, direction4, direct;
	int directions1, directions2, directions3, directions4, directs;
	AStar::Vec3i temp;//��ʱ�洢Ԥ���
	direction1 = {
			{ 0, 1, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { -1, 0, 0 },
			{ -1, -1, 0 }, { 1, 1, 0 }, { -1, 1, 0 }, { 1, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }
	};
	direction2 = { { 0, 0, 1 }
	};
	direction3 = {
			{ 0, 1, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { -1, 0, 0 },
			{ -1, -1, 0 }, { 1, 1, 0 }, { -1, 1, 0 }, { 1, -1, 0 }, { 0, 0, -1 }
	};
	direction4 = {
			{ 0, 1, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { -1, 0, 0 },
			{ -1, -1, 0 }, { 1, 1, 0 }, { -1, 1, 0 }, { 1, -1, 0 }, { 0, 0, 1 }
	};
	directions1 = 10;
	directions2 = 1;
	directions3 = 9;
	directions4 = 9;

	if (pstMap->nHHigh > uav.nZ&& uav.nZ > pstMap->nHLow){
		direct = direction1;
		directs = directions1;
	}
	else if (uav.nZ < pstMap->nHLow){
		direct = direction2;
		directs = directions2;
	}
	else if (uav.nZ == pstMap->nHHigh){
		direct = direction3;
		directs = directions3;
	}
	else if (uav.nZ == pstMap->nHLow)
	{
		direct = direction4;
		directs = directions4;
	}
	for (int i = 0; i < directs; i++)
	{
		temp.x = uav.nX + direct[i].x;
		temp.y = uav.nY + direct[i].y;
		temp.z = uav.nZ + direct[i].z;
		if (temp.x < 0 || temp.x >= pstMap->nMapX ||
			temp.y < 0 || temp.y >= pstMap->nMapY ||
			temp.z < 0 || temp.z >= pstMap->nMapZ ||
			generator.map[temp.x][temp.y][temp.z] == true)
		{
			continue;
		}
		detetDirection.push_back(temp);
	}
	return 	detetDirection;
}

void SpreadOut(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, UAV *uav ,int random)
{
	AStar::Vec3i startPoint = { uav->nX, uav->nY, uav->nZ };
	AStar::Vec3i endPoint;
	AStar::CoordinateList nextPoints = direction(*uav, pstMap);
	int numNext = nextPoints.size();
	if (numNext==0)
	{
		endPoint = startPoint;
	}
	else
	{
		int derection = random %numNext;
		switch (derection)
		{
		case 0:
			endPoint = nextPoints[0];
			break;
		case 1:
			endPoint = nextPoints[1];
			break;
		case 2:
			endPoint = nextPoints[2];
			break;
		case 3:
			endPoint = nextPoints[3];
			break;
		case 4:
			endPoint = nextPoints[4];
			break;
		case 5:
			endPoint = nextPoints[5];
			break;
		case 6:
			endPoint = nextPoints[6];
			break;
		case 7:
			endPoint = nextPoints[7];
			break;
		case 8:
			endPoint = nextPoints[8];
			break;
		case 9:
			endPoint = nextPoints[9];
		default:
			break;
		}
	}
	int CollisionType2 = 0;
	for (int j = 0; j < pstMatch->nUavWeNum; j++)
	{
		AStar::Vec3i jUavStartPoint = { pstMatch->astWeUav[j].nX, pstMatch->astWeUav[j].nY, pstMatch->astWeUav[j].nZ };
		AStar::Vec3i jUavEndPoint = { pstFlayPlane->astUav[j].nX, pstFlayPlane->astUav[j].nY, pstFlayPlane->astUav[j].nZ };
		if (pstFlayPlane->astUav[j].haveDeal == false)//û��������
			continue;
		else
		{
			if (1 == IsCollision(jUavStartPoint, jUavEndPoint, startPoint, endPoint))//�����ײ1
			{
				CollisionType2 = 1;
				uav->haveDeal = true;
				break;

			}
			else if (2 == IsCollision(jUavStartPoint, jUavEndPoint, startPoint, endPoint))
			{
				generator.addCollision(jUavStartPoint);
				SpreadOut(pstMap, pstMatch, pstFlayPlane, uav,random+1);
				CollisionType2 = 2;
				generator.removeCollision(jUavStartPoint);
				break;
			}
			else if (3 == IsCollision(jUavStartPoint, jUavEndPoint, startPoint, endPoint))
			{
				generator.addCollision(jUavEndPoint);
				generator.addCollision(endPoint);
				SpreadOut(pstMap, pstMatch, pstFlayPlane, uav,random+1);
				CollisionType2 = 3;
				generator.removeCollision(jUavEndPoint);
				generator.removeCollision(endPoint);
				break;
			}
		}

	}
	if (CollisionType2 == 0 )
	{
		uav->nX = endPoint.x;
		uav->nY = endPoint.y;
		uav->nZ = endPoint.z;
		uav->haveDeal = true;
	}
	
}

void avoid(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, UAV *uav, int random)
{
	AStar::Vec3i startPoint = { uav->nX, uav->nY, uav->nZ };
	AStar::Vec3i endPoint;
	AStar::CoordinateList nextPoints = direction(*uav, pstMap);
	int numNext = nextPoints.size();
	if (numNext == 0)
	{
		endPoint = startPoint;
	}
	else
	{
		int derection = random %numNext;
		switch (derection)
		{
		case 0:
			endPoint = nextPoints[0];
			break;
		case 1:
			endPoint = nextPoints[1];
			break;
		case 2:
			endPoint = nextPoints[2];
			break;
		case 3:
			endPoint = nextPoints[3];
			break;
		case 4:
			endPoint = nextPoints[4];
			break;
		case 5:
			endPoint = nextPoints[5];
			break;
		case 6:
			endPoint = nextPoints[6];
			break;
		case 7:
			endPoint = nextPoints[7];
			break;
		case 8:
			endPoint = nextPoints[8];
			break;
		case 9:
			endPoint = nextPoints[9];
		default:
			break;
		}
	}
	int CollisionType2 = 0;
	for (int j = 0; j < pstMatch->nUavWeNum; j++)
	{
		AStar::Vec3i jUavStartPoint = { pstMatch->astWeUav[j].nX, pstMatch->astWeUav[j].nY, pstMatch->astWeUav[j].nZ };
		AStar::Vec3i jUavEndPoint = { pstFlayPlane->astUav[j].nX, pstFlayPlane->astUav[j].nY, pstFlayPlane->astUav[j].nZ };
		if (pstFlayPlane->astUav[j].haveDeal == false)//û��������
			continue;
		else
		{
			if (1 == IsCollision(jUavStartPoint, jUavEndPoint, startPoint, endPoint))//�����ײ1
			{
				generator.addCollision(jUavEndPoint);
				avoid(pstMap, pstMatch, pstFlayPlane, uav, random + 1);
				CollisionType2 = 1;
				generator.removeCollision(jUavStartPoint);
				break;

			}
			else if (2 == IsCollision(jUavStartPoint, jUavEndPoint, startPoint, endPoint))
			{
				generator.addCollision(jUavStartPoint);
				avoid(pstMap, pstMatch, pstFlayPlane, uav, random + 1);
				CollisionType2 = 2;
				generator.removeCollision(jUavStartPoint);
				break;
			}
			else if (3 == IsCollision(jUavStartPoint, jUavEndPoint, startPoint, endPoint))
			{
				generator.addCollision(jUavEndPoint);
				generator.addCollision(endPoint);
				avoid(pstMap, pstMatch, pstFlayPlane, uav, random + 1);
				CollisionType2 = 3;
				generator.removeCollision(jUavEndPoint);
				generator.removeCollision(endPoint);
				break;
			}
		}

	}
	if (CollisionType2 == 0)
	{
		uav->nX = endPoint.x;
		uav->nY = endPoint.y;
		uav->nZ = endPoint.z;
		uav->haveDeal = true;
	}
}
//Ԥ������֮��Ĳ���
int EstimateSteps(AStar::Vec3i startPoint, AStar::Vec3i endPoint, MAP_INFO *pstMap)
{
	int xsteps = abs(startPoint.x - endPoint.x);
	int ysteps = abs(startPoint.y - endPoint.y);
	int xysteps = xsteps > ysteps ? xsteps : ysteps;
	int steps;
	if (xysteps == 0)
		steps = abs(startPoint.z - endPoint.z);
	else if (xysteps != 0 && endPoint.z >= pstMap->nHLow&&startPoint.z >= pstMap->nHLow)
		steps = xysteps + abs(startPoint.z - endPoint.z);
	else
		steps = xysteps + abs(startPoint.z - pstMap->nHLow) + abs(pstMap->nHLow - endPoint.z);
	return steps;
}
bool cmp(const UAV &x, const UAV &y)
{
	if (x.uavValue != y.uavValue)
		return x.uavValue<y.uavValue;
	else
		return x.remain_electricity<y.remain_electricity;
}
void ConnectWeAndEnemy(GOODS enemyGoods, UAV enemyUav, FLAY_PLANE *pstFlayPlane, MAP_INFO *pstMap)
{
	//����enemyUav��enemyGoods���õĲ���
	int stepEnnmy;
	AStar::Vec3i endPoint = { enemyGoods.nEndX, enemyGoods.nEndY, pstMap->nHLow - 1 };
	if (enemyUav.nGoodsNo == -1)
	{
		int step1 = enemyUav.nZ;
		AStar::Vec3i startPoint = { enemyGoods.nStartX, enemyGoods.nStartY, 0 };
		AStar::CoordinateList apath;
		MinStepsCal(startPoint, endPoint, &apath);
		int step2 = apath.size();
		stepEnnmy = step1 + step2;
	}
	else
	{
		AStar::Vec3i startPoint = { enemyUav.nX, enemyUav.nY, enemyUav.nZ };
		AStar::CoordinateList apath;
		MinStepsCal(startPoint, endPoint, &apath);
		stepEnnmy = apath.size();
	}
	

	//Ѱ�Һ��ʵķɻ�������
	std::vector<UAV> AvailableTable;
	for (int i = 0; i <pstFlayPlane->nUavNum; i++)
	{
		if (pstFlayPlane->astUav[i].nStatus == UAV_CRASH)
			continue;
		if (pstFlayPlane->astUav[i].task == 0 || pstFlayPlane->astUav[i].task == 1)
		{
			//���Ǽ�ֵ
			/*if (pstFlayPlane->astUav[i].szType[1] != cheapestUav.szType[1])
				continue;*/

			AStar::Vec3i WeUavPoint = { pstFlayPlane->astUav[i].nX, pstFlayPlane->astUav[i].nY, pstFlayPlane->astUav[i].nZ };
			//���ǲ���
			int EsSteps = EstimateSteps(WeUavPoint, endPoint, pstMap);
			if (EsSteps < stepEnnmy - 5)//5������
			{
				AvailableTable.push_back(pstFlayPlane->astUav[i]);
			}
		}
	}
	//��AvailableTable����һ�����򣬸��ݼ�ֵ��С�͵���
	std::sort(AvailableTable.begin(), AvailableTable.end(), cmp);
	//ȡ��������������һ�μ���Ƿ����
	for (int i = 0; i < AvailableTable.size(); i++)
	{
		//��֤����
		AStar::Vec3i iStartPoint = { AvailableTable[i].nX, AvailableTable[i].nY, AvailableTable[i].nZ };
		AStar::CoordinateList iPath;
		MinStepsCal(iStartPoint, endPoint, &iPath);
		int iStep = iPath.size();
		if (iStep <= stepEnnmy - 3 && AvailableTable[i].uavValue<enemyUav.uavValue)
		{
			AvailableTable[i].task = 4;
			AvailableTable[i].path = iPath;
			AvailableTable[i].targetGood = enemyGoods;
			AvailableTable[i].AttackUavNo = enemyUav.nNO;
			for (int j = 0; j < pstFlayPlane->nUavNum; j++)
			{
				if (pstFlayPlane->astUav[j].nNO == AvailableTable[i].nNO)
				{
					pstFlayPlane->astUav[j] = AvailableTable[i];
					break;
				}
			}
			break;
		}
	}
}

void cashEnemy(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane)
{
	bool flag;//false��ʾû���б��true��ʾ���б���
	for (int i = 0; i < pstMatch->nUavEnemyNum; i++)
	{
		flag = false;
		if (pstMatch->astEnemyUav[i].nX == enemyParkingX&&pstMatch->astEnemyUav[i].nY == enemyParkingY)
			continue;
		if (pstMatch->astEnemyUav[i].nGoodsNo != -1)
		{
			for (int j = 0; j <pstFlayPlane->nUavNum; j++)
			{
				if ((pstFlayPlane->astUav[j].task == 4 && pstFlayPlane->astUav[j].nStatus != UAV_CRASH) || (pstFlayPlane->astUav[j].task == 7 && pstFlayPlane->astUav[j].nStatus != UAV_CRASH))
				{
					if (pstFlayPlane->astUav[j].AttackUavNo == pstMatch->astEnemyUav[i].nNO)
					{
						flag = true;
						break;
					}
				}
			}
			if (flag == true)
				continue;
			for (int k = 0; k < pstMatch->nGoodsNum; k++)
			{
				if (pstMatch->astGoods[k].nNO == pstMatch->astEnemyUav[i].nGoodsNo)
				{
					//���з��ɻ���һ��������
					if (pstMatch->astEnemyUav[i].nX == pstMatch->astGoods[k].nEndX&&pstMatch->astEnemyUav[i].nY == pstMatch->astGoods[k].nEndY)
					{
					}
					else
					{
						ConnectWeAndEnemy(pstMatch->astGoods[k], pstMatch->astEnemyUav[i], pstFlayPlane, pstMap);
					}
					break;
				}
			}	
		}
		else if (pstMatch->astEnemyUav[i].nGoodsNo == -1&&pstMatch->astEnemyUav[i].nZ < pstMap->nHLow)
		{
			for (int j = 0; j <pstFlayPlane->nUavNum; j++)
			{
				if ((pstFlayPlane->astUav[j].task == 4 && pstFlayPlane->astUav[j].nStatus != UAV_CRASH) || (pstFlayPlane->astUav[j].task == 7 && pstFlayPlane->astUav[j].nStatus != UAV_CRASH))
				{
					if (pstFlayPlane->astUav[j].AttackUavNo == pstMatch->astEnemyUav[i].nNO)
					{
						flag = true;
						break;
					}
				}
			}
			if (flag == true)
				continue;
			for (int j = 0; j < pstFlayPlane->nUavNum; j++)
			{
				if (pstMatch->astEnemyUav[i].nX == pstFlayPlane->astUav[j].nX&&pstMatch->astEnemyUav[i].nY == pstFlayPlane->astUav[j].nY)
				{
					flag = true;
					break;
				}
			}
			if (flag == true)
				continue;
			for (int k = 0; k < pstMatch->nGoodsNum; k++)
			{
				if (pstMatch->astGoods[k].nStartX == pstMatch->astEnemyUav[i].nX&&pstMatch->astGoods[k].nStartY == pstMatch->astEnemyUav[i].nY)
				{
					//���з��ɻ���һ��������
					ConnectWeAndEnemy(pstMatch->astGoods[k], pstMatch->astEnemyUav[i], pstFlayPlane, pstMap);
					break;
				}
			}
		}
		
	}

}




void MaintainValue(MAP_INFO *pstMap, FLAY_PLANE *pstFlayPlane, MATCH_STATUS * pstMatch)
{
	//�ҷ��ɻ��ļ�ֵ
	for (int i = 0; i < pstFlayPlane->nUavNum; i++)
	{
		int uavValue = 0;
		int goodsValue = 0;
		//�ɻ���ֵ
		for (int j = 0; j < pstMap->nUavPriceNum; j++)
		{
			if (pstMap->astUavPrice[j].szType[1] == pstFlayPlane->astUav[i].szType[1])
			{
				uavValue = pstMap->astUavPrice[j].nValue;
				break;
			}
		}
		//��Ʒ��ֵ
		if (pstFlayPlane->astUav[i].nGoodsNo==-1&&pstFlayPlane->astUav[i].task==1)
		{
			for (int j = 0; j < pstMatch->nGoodsNum; j++)
			{
				if (pstMatch->astGoods[j].nNO == pstFlayPlane->astUav[i].targetGood.nNO)
				{
					int targetGoodsValue = pstMatch->astGoods[j].nValue;
					AStar::Vec3i uavPoint = { pstFlayPlane->astUav[i].nX, pstFlayPlane->astUav[i].nY, pstFlayPlane->astUav[i].nZ };
					AStar::Vec3i targetGoodsPoint = { pstMatch->astGoods[j].nStartX, pstMatch->astGoods[j].nStartY,0};
					int targetSteps=EstimateSteps(uavPoint, targetGoodsPoint, pstMap);
					goodsValue = targetGoodsValue / pow(1.05,targetSteps);
				}
			}
		}
		else 
		{
			for (int j = 0; j < pstMatch->nGoodsNum; j++)
			{
				if (pstMatch->astGoods[j].nNO == pstFlayPlane->astUav[i].nGoodsNo)
				{
					goodsValue = pstMatch->astGoods[j].nValue;
				}
			}
		}
		pstFlayPlane->astUav[i].uavValue = uavValue;
		pstFlayPlane->astUav[i].goodsValue = goodsValue;
		pstFlayPlane->astUav[i].totalValue == uavValue + goodsValue;
	}

	//�з��ɻ��ļ�ֵ
	for (int i = 0; i <pstMatch->nUavEnemyNum; i++)
	{
		int uavValue = 0;
		int goodsValue = 0;
		//�ɻ���ֵ
		for (int j = 0; j < pstMap->nUavPriceNum; j++)
		{
			if (pstMap->astUavPrice[j].szType[1] == pstMatch->astEnemyUav[i].szType[1])
			{
				uavValue = pstMap->astUavPrice[j].nValue;
				break;
			}
		}
		//��Ʒ��ֵ
		for (int j = 0; j < pstMatch->nGoodsNum; j++)
		{
			if (pstMatch->astGoods[j].nNO == pstMatch->astEnemyUav[i].nGoodsNo)
			{
				goodsValue = pstMatch->astGoods[j].nValue;
			}
		}
		pstMatch->astEnemyUav[i].uavValue = uavValue;
		pstMatch->astEnemyUav[i].goodsValue = goodsValue;
		pstMatch->astEnemyUav[i].totalValue == uavValue + goodsValue;
	}
}

void toEnemyParking(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, UAV *uav)
{
	
	AStar::Vec3i enemyParking = { enemyParkingX, enemyParkingY, 1 };
	AStar::Vec3i uavPoint = { uav->nX, uav->nY, uav->nZ };
	AStar::CoordinateList path;
	MinStepsCal(uavPoint,enemyParking, &path);
	move2point(pstMatch, pstFlayPlane, path, uav);
	uav->task = 3;
	uav->AttackUavNo = -1;
	uav->path = path;
	uav->haveDeal = true;

}

void charge(UAV *uav, MAP_INFO *pstMap)
{
	for (int j = 0; j < pstMap->nUavPriceNum; j++)
	{
		if (uav->szType[1] == pstMap->astUavPrice[j].szType[1])
		{
			uav->remain_electricity += pstMap->astUavPrice[j].charge;
			uav->task = 5;//5������
			if (uav->remain_electricity > pstMap->astUavPrice[j].capacity)
			{
				uav->remain_electricity = pstMap->astUavPrice[j].capacity;
				uav->IsFull = true;
			}
			uav->haveDeal = true;
			break;
		}
	}
}

bool cmp1(const UAV &x, const UAV &y)
{
	return x.uavValue>y.uavValue;
}

void Track(MATCH_STATUS * pstMatch, UAV *uav, FLAY_PLANE *pstFlayPlane)
{
	AStar::Vec3i uavPoint = { uav->nX, uav->nY, uav->nZ };
	AStar::Vec3i enemtUavPoint ;
	bool havefund = false;
	for (int i = 0; i < pstMatch->nUavEnemyNum; i++)
	{
		if (pstMatch->astEnemyUav[i].nNO == uav->AttackUavNo)
		{
			havefund = true;
			if (pstMatch->astEnemyUav[i].nStatus == UAV_FOG)
			{
				if (uav->path.size() == 0)
				{
					enemtUavPoint = {0,0,0};
				}
				else
				{
					enemtUavPoint = uav->path[0];
				}
			
			}
			else if (pstMatch->astEnemyUav[i].nStatus == UAV_NOMAL || pstMatch->astEnemyUav[i].nStatus == UAV_CHARGING)
			{
				enemtUavPoint.x = pstMatch->astEnemyUav[i].nX;
				enemtUavPoint.y = pstMatch->astEnemyUav[i].nY;
				enemtUavPoint.z = pstMatch->astEnemyUav[i].nZ;
			}
			break;
		}
	}
	if (havefund == true)
	{
		AStar::CoordinateList path;
		MinStepsCal(uavPoint, enemtUavPoint, &path);
		move2point(pstMatch, pstFlayPlane, path, uav);
		uav->path = path;
		uav->haveDeal = true;
	}
	else
	{
		uav->task = 0;
	}
}

void InitDisInfo(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane)
{
	for (int i = 0; i < pstFlayPlane->nUavNum; i++)
	{
		if (pstFlayPlane->astUav[i].uavValue <= cheapestUav.nValue)
			continue;
		std::vector<DIS_INFO> temp;
		for (int j = 0; j < pstMatch->nUavEnemyNum; j++)
		{
			if (pstMatch->astEnemyUav[j].uavValue != cheapestUav.nValue)
				continue;
			//�����ҷ���з��ľ���
			AStar::CoordinateList apath;
			AStar::Vec3i startPoint = { pstMatch->astEnemyUav[j].nX, pstMatch->astEnemyUav[j].nY, pstMatch->astEnemyUav[j].nZ};
			AStar::Vec3i endPoint = { pstFlayPlane->astUav[i].nX, pstFlayPlane->astUav[i].nY, pstFlayPlane->astUav[i].nZ };
			MinStepsCal(startPoint, endPoint, &apath);
			int Distance = apath.size();

			DIS_INFO aDisInfo;
			aDisInfo.diastance = Distance;
			aDisInfo.eneMyUavNO = pstMatch->astEnemyUav[j].nNO;
			aDisInfo.weUavNO = pstFlayPlane->astUav[i].nNO;
			aDisInfo.reduceTimes = 0;
			temp.push_back(aDisInfo);

		}
		disInfo.push_back(temp);
	}
}
bool cmp2(const DIS_INFO &x, const DIS_INFO &y)
{
	if (x.reduceTimes != y.reduceTimes)
		return x.diastance<y.diastance;
	else
		return x.reduceTimes>y.reduceTimes;
}
void newDisInfo(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane)
{
	for (int i = 0; i < pstFlayPlane->nUavNum; i++)
	{
		if (pstFlayPlane->astUav[i].uavValue <= cheapestUav.nValue)
			continue;
		if (pstFlayPlane->astUav[i].nStatus == UAV_CRASH)
			continue;
		std::vector<DIS_INFO> temp;
		for (int j = 0; j < pstMatch->nUavEnemyNum; j++)
		{
			if (pstMatch->astEnemyUav[j].uavValue != cheapestUav.nValue)
				continue;
			//�����ҷ���з��ľ���
			AStar::Vec3i startPoint = { pstMatch->astEnemyUav[j].nX, pstMatch->astEnemyUav[j].nY, pstMatch->astEnemyUav[j].nZ };
			AStar::Vec3i endPoint = { pstFlayPlane->astUav[i].nX, pstFlayPlane->astUav[i].nY, pstFlayPlane->astUav[i].nZ };
			int Distance = EstimateSteps(startPoint, endPoint, pstMap);
			DIS_INFO aDisInfo;
			bool IsNewEnemy = true;
			int lindex = 0;
			for (int l = 0; l < disInfo.size(); l++)
			{
				if (disInfo[l][0].weUavNO == pstFlayPlane->astUav[i].nNO)
				{
					lindex = l;
					for (int k = 0; k < disInfo[l].size(); k++)
					{
						if (disInfo[l][k].weUavNO == pstFlayPlane->astUav[i].nNO&&disInfo[l][k].eneMyUavNO == pstMatch->astEnemyUav[j].nNO)
						{
							if (pstFlayPlane->astUav[i].nX == pstMap->nParkingX&&pstFlayPlane->astUav[i].nY == pstMap->nParkingY&&pstFlayPlane->astUav[i].nZ <pstMap->nHLow)
							{
								disInfo[l][k].diastance = Distance;
							}
							else
							{
								if (disInfo[l][k].diastance >= Distance)
									disInfo[l][k].reduceTimes += 1;
								else
									disInfo[l][k].reduceTimes = 0;
								disInfo[l][k].diastance = Distance;
							}
							IsNewEnemy = false;
							break;
						}
					}
					break;
				}
			}
			
			if (IsNewEnemy == true)
			{
				aDisInfo.eneMyUavNO = pstMatch->astEnemyUav[j].nNO;
				aDisInfo.weUavNO = pstFlayPlane->astUav[i].nNO;
				aDisInfo.reduceTimes = 0;
				aDisInfo.diastance = Distance;
				disInfo[lindex].push_back(aDisInfo);
			}

		}
		
	}

}

/** @fn     void AlgorithmCalculationFun()
*  @brief	ѧ�����㷨���㣬 ����ʲô�Ķ��Լ�д��
*	@return void
*/
void  AlgorithmCalculationFun(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane)
{

	//ʹ��pstMatch��pstFlayPlane����
	pstFlayPlane->nUavNum = pstMatch->nUavWeNum;
	for (int i = 0; i < pstMatch->nUavWeNum; i++)
	{
		pstFlayPlane->astUav[i].nGoodsNo = pstMatch->astWeUav[i].nGoodsNo;
		pstFlayPlane->astUav[i].nLoadWeight = pstMatch->astWeUav[i].nLoadWeight;
		pstFlayPlane->astUav[i].nNO = pstMatch->astWeUav[i].nNO;
		pstFlayPlane->astUav[i].nStatus = pstMatch->astWeUav[i].nStatus;
		pstFlayPlane->astUav[i].nX = pstMatch->astWeUav[i].nX;
		pstFlayPlane->astUav[i].nY = pstMatch->astWeUav[i].nY;
		pstFlayPlane->astUav[i].nZ = pstMatch->astWeUav[i].nZ;
		pstFlayPlane->astUav[i].szType[0] = pstMatch->astWeUav[i].szType[0];
		pstFlayPlane->astUav[i].szType[1] = pstMatch->astWeUav[i].szType[1];
		pstFlayPlane->astUav[i].remain_electricity = pstMatch->astWeUav[i].remain_electricity;
	}
	MaintainValue(pstMap, pstFlayPlane, pstMatch);//����ÿһ���ɻ��ļ�ֵ���ɻ�����Ʒ���ܼ�ֵ��,�����ҷ��ɻ��ļ�ֵ������pstFlayPlane���з��ɻ��ļ�ֵ������ pstMatch��
	for (int i = 0; i < pstMatch->nUavWeNum; i++)
	{
		pstFlayPlane->astUav[i].haveDeal = false;
		if (pstFlayPlane->astUav[i].task == 1 || pstFlayPlane->astUav[i].task==3)
		{
			pstFlayPlane->astUav[i].task = 0;
		}
	}
	for (int i = 0; i < MAX_UAV_PRICE_NUM; i++)
	{
		pstFlayPlane->szPurchaseType[i][0] = '\0';
		pstFlayPlane->szPurchaseType[i][1] = '\0';
	}
	pstFlayPlane->nPurchaseNum = 0;
	//�ڻ���û�г������Ҫ���

	for (int i = 0; i < pstFlayPlane->nUavNum; i++)
	{
		if (pstFlayPlane->astUav[i].nX != pstMap->nParkingX || pstFlayPlane->astUav[i].nY != pstMap->nParkingY || pstFlayPlane->astUav[i].nZ != 0)
			continue;
		if (pstFlayPlane->astUav[i].IsFull == false)
			charge(&pstFlayPlane->astUav[i], pstMap);
		else
			pstFlayPlane->astUav[i].task = 0;
	}
	if (pstMatch->nTime == 126)
	{
		int ll = 0;
	}
	if (pstMatch->nTime ==1)
	{
		InitDisInfo(pstMap, pstMatch, pstFlayPlane);
	
	}
	if (pstMatch->nTime != 1)
	{
		newDisInfo(pstMap, pstMatch, pstFlayPlane);
		int kk = 0;
	}

	//�жϷɻ���񱻸���
	//���û�п���ײ�ٵ����
	for (int i = 0; i < disInfo.size(); i++)
	{

		std::sort(disInfo[i].begin(), disInfo[i].end(), cmp2);
		if (disInfo[i][0].reduceTimes > 6 && disInfo[i][0].diastance <= 10 && pstFlayPlane->astUav[disInfo[i][0].weUavNO].nGoodsNo==-1)
		{
			pstFlayPlane->astUav[disInfo[i][0].weUavNO].task = 8;
			pstFlayPlane->astUav[disInfo[i][0].weUavNO].AttackUavNo = disInfo[i][0].eneMyUavNO;
		}
		else
		{
			if (pstFlayPlane->astUav[disInfo[i][0].weUavNO].task == 8)
				pstFlayPlane->astUav[disInfo[i][0].weUavNO].task = 0;
		}
	}


	//���
	srand((unsigned)time(NULL));
	for (int i = 0; i < pstFlayPlane->nUavNum; i++)
	{
		if (pstFlayPlane->astUav[i].task != 8)
			continue;
		if (pstFlayPlane->astUav[i].nStatus==UAV_CRASH)
			continue;
		if (pstFlayPlane->astUav[i].uavValue <= cheapestUav.nValue)
			continue;
		if (pstFlayPlane->astUav[i].nZ < pstMap->nHLow)
			continue;
		bool enemyHaveCrash = true;
		for (int j = 0; j < pstMatch->nUavEnemyNum; j++)
		{
			if (pstMatch->astEnemyUav[j].nNO == pstFlayPlane->astUav[i].AttackUavNo)
			{
				AStar::Vec3i start= { pstFlayPlane->astUav[i].nX, pstFlayPlane->astUav[i].nY, pstFlayPlane->astUav[i].nZ };
				AStar::Vec3i end = { pstMatch->astEnemyUav[j].nX, pstMatch->astEnemyUav[j].nY, pstMatch->astEnemyUav[j].nZ };
				AStar::CoordinateList path;
				MinStepsCal(start, end, &path);
				int step = path.size()-1;
				if (step>1)
				{
					if (pstFlayPlane->astUav[i].nZ>=pstMap->nHLow)
						pstFlayPlane->astUav[i].haveDeal = true;
					else
					{
						generator.addCollision(end);
						int ran = rand();
						avoid(pstMap, pstMatch, pstFlayPlane, &pstFlayPlane->astUav[i], ran);
						generator.removeCollision(end);
						pstFlayPlane->astUav[i].haveDeal = true;
					}
				}
				else if (step==1)
				{
					generator.addCollision(end);
					int ran = rand();
					avoid(pstMap, pstMatch, pstFlayPlane, &pstFlayPlane->astUav[i], ran);
					generator.removeCollision(end);
					pstFlayPlane->astUav[i].haveDeal = true;
				}
				enemyHaveCrash = false;
				break;
			}
		}
		if (enemyHaveCrash == true)
		{
			pstFlayPlane->astUav[i].task = 0;
		}
		
	}
	

	//�ҵ��˷ɻ���ƷĿ�ĵ�
	cashEnemy(pstMap, pstMatch, pstFlayPlane);

	//����
	for (int j = 0; j < pstFlayPlane->nUavNum; j++)
	{
		if (pstFlayPlane->astUav[j].task != 4)
			continue;
		if (pstFlayPlane->astUav[j].nStatus == UAV_CRASH)
			continue;
	/*	if (pstFlayPlane->astUav[j].szType[1] != cheapestUav.szType[1])
			continue;*/
		bool enemyCash = true;
		for (int i = 0; i < pstMatch->nUavEnemyNum; i++)
		{
			if (pstFlayPlane->astUav[j].AttackUavNo == pstMatch->astEnemyUav[i].nNO)
			{
				enemyCash = false;
				break;
			}
		}
		if (enemyCash == false)
		{
			move2point(pstMatch, pstFlayPlane, pstFlayPlane->astUav[j].path, &pstFlayPlane->astUav[j]);
			pstFlayPlane->astUav[j].haveDeal = true;
		}
		else
		{
			pstFlayPlane->astUav[j].task = 0;
		}
	}



	//�ҵ����˵Ĵ�ɻ�N0
	std::vector<UAV> EnemyTable;
	for (int  i = 0; i < pstMatch->nUavEnemyNum; i++)
	{
		EnemyTable.push_back(pstMatch->astEnemyUav[i]);
	}
	//��enemyTable���ݷɻ���ֵ��С��������
	std::sort(EnemyTable.begin(), EnemyTable.end(), cmp1);
	for (int j = 0;j < EnemyTable.size(); j++)
	{
		if (EnemyTable[j].uavValue <= cheapestUav.nValue)
			continue;
		bool haveAttack = false;
		for (int i = 0; i < pstFlayPlane->nUavNum; i++)
		{
			if (pstFlayPlane->astUav[i].nStatus == UAV_CRASH)
				continue;
			if (pstFlayPlane->astUav[i].task == 7 || pstFlayPlane->astUav[i].task == 4)
			{
				if (pstFlayPlane->astUav[i].AttackUavNo == EnemyTable[j].nNO)
				{
					haveAttack = true;
					break;
				}
			}
		}
		if (haveAttack == true)
			continue;
		for (int i = 0; i < pstFlayPlane->nUavNum; i++)
		{
			if (pstFlayPlane->astUav[i].szType[1] != cheapestUav.szType[1])
				continue;
			if (pstFlayPlane->astUav[i].nStatus == UAV_CRASH)
				continue;
			if (pstFlayPlane->astUav[i].task !=0)
				continue;
			pstFlayPlane->astUav[i].AttackUavNo = EnemyTable[j].nNO;
			pstFlayPlane->astUav[i].task = 7;//task==7����׷�ٵ���
			break;
		}
	}
	//����task==7�ķɻ�
	for (int i = 0; i < pstFlayPlane->nUavNum; i++)
	{
		if (pstFlayPlane->astUav[i].task != 7)
			continue;
		if (pstFlayPlane->astUav[i].nStatus == UAV_CRASH)
			continue;
		Track(pstMatch, &pstFlayPlane->astUav[i], pstFlayPlane);
		/*for (int j = 0; j < pstMatch->nUavEnemyNum; j++)
		{
			if (pstMatch->astEnemyUav[j].nNO == pstFlayPlane->astUav[i].AttackUavNo)
			{
				if (pstMatch->astEnemyUav[j].nGoodsNo != -1)
				{
					pstFlayPlane->astUav[i].task = 4;
					AStar::Vec3i startPoint = { pstFlayPlane->astUav[i].nX, pstFlayPlane->astUav[i].nY, pstFlayPlane->astUav[i].nZ };
					AStar::Vec3i endPoint;
					for (int k = 0; k < pstMatch->nGoodsNum; k++)
					{
						if (pstMatch->astGoods[k].nNO == pstMatch->astEnemyUav[j].nGoodsNo)
						{
							endPoint.x = pstMatch->astGoods[k].nEndX;
							endPoint.y = pstMatch->astGoods[k].nEndY;
							endPoint.z = pstMap->nHLow-1;
							pstFlayPlane->astUav[i].targetGood = pstMatch->astGoods[k];
							break;
						}
					}
					AStar::CoordinateList path7;
					MinStepsCal(startPoint, endPoint, &path7);
					pstFlayPlane->astUav[i].path = path7;
				}
				else
				{
					Track(pstMatch, &pstFlayPlane->astUav[i], pstFlayPlane);
				}
				break;
			}
		}*/
	}


	
	//����2�ͻ�
	for (int i = 0; i < pstFlayPlane->nUavNum; i++)
	{
		if (pstFlayPlane->astUav[i].task != 2)
				continue;
		if (pstFlayPlane->astUav[i].nStatus == UAV_CRASH)
				continue;
		if (pstFlayPlane->astUav[i].nGoodsNo ==-1)
			continue;
		SendGood(pstMatch, pstFlayPlane, &pstFlayPlane->astUav[i], pstFlayPlane->astUav[i].path);
		pstFlayPlane->astUav[i].haveDeal = true;
		//�����Ʒ������
		int weight = 0;
		for (int j= 0; j < pstMatch->nGoodsNum; j++)
		{
			if (pstMatch->astGoods[j].nNO == pstFlayPlane->astUav[i].nGoodsNo)
			{
				weight = pstMatch->astGoods[j].nWeight;
				break;
			}
		}
		pstFlayPlane->astUav[i].remain_electricity -= weight;
	}

	//����6ȡ��
	for (int i = 0; i < pstFlayPlane->nUavNum; i++)
	{
		if (pstFlayPlane->astUav[i].task != 6)
			continue;
		if (pstFlayPlane->astUav[i].nStatus == UAV_CRASH)
			continue;
		//������ֱ��ҷ����ٵĵ��˷ɻ�ȥȥ������Ҫȡ�Ļ����ô�Ͱ��������ó�0
		for (int j = 0; j < pstMatch->nUavEnemyNum; j++)
		{
			if (pstMatch->astEnemyUav[j].nX == pstFlayPlane->astUav[i].nX&&pstMatch->astEnemyUav[j].nY == pstFlayPlane->astUav[i].nY&&pstMatch->astEnemyUav[j].nZ < pstFlayPlane->astUav[i].nZ)
			{
				for (int k = 0; k < pstFlayPlane->nUavNum; k++)
				{
					if ((pstFlayPlane->astUav[k].task == 7 && pstFlayPlane->astUav[k].AttackUavNo == pstMatch->astEnemyUav[j].nNO&&pstFlayPlane->astUav[k].nStatus != UAV_CRASH) 
						|| (pstFlayPlane->astUav[k].task == 4 && pstFlayPlane->astUav[k].AttackUavNo == pstMatch->astEnemyUav[j].nNO&&pstFlayPlane->astUav[k].nStatus != UAV_CRASH))
					{
						pstFlayPlane->astUav[i].task = 0;
						break;
					}
				}
				break;
			}
		}
		if (pstFlayPlane->astUav[i].task == 6)
		{
			GetGood(pstMatch, pstFlayPlane, &pstFlayPlane->astUav[i].targetGood, &pstFlayPlane->astUav[i], pstFlayPlane->astUav[i].path, pstFlayPlane->astUav[i].pathNext);
			pstFlayPlane->astUav[i].haveDeal = true;
		}
	}

	//������ֱ��������Ʒ�ľ���ͼ�ֵ

	std::vector<std::vector<COST_PERF>> costPerfTable;

	for (int i_good = 0; i_good < pstMatch->nGoodsNum; i_good++)
	{
		//ȷ���˻������ֱ�Ӱѵڶ��εĲ����������
		if (pstMatch->astGoods[i_good].nState != 0)
			continue;
		bool haveUavTo = false;
		for (int i = 0; i < pstFlayPlane->nUavNum; i++)
		{
			if ((pstFlayPlane->astUav[i].task == 6 && pstFlayPlane->astUav[i].targetGood.nNO == pstMatch->astGoods[i_good].nNO)
				|| (pstFlayPlane->astUav[i].task == 1 && pstFlayPlane->astUav[i].targetGood.nNO == pstMatch->astGoods[i_good].nNO)
				|| (pstFlayPlane->astUav[i].task == 2 && pstFlayPlane->astUav[i].nGoodsNo == pstMatch->astGoods[i_good].nNO))
			{
				haveUavTo = true;
				break;
			}
		}
		if (haveUavTo == true)
			continue;
		//������ֻ����Ϸ��Ѿ����ֵ��˵ķɻ������˵ķɻ��Ѿ�������ˣ�ֱ��continnue
		bool cantGet = false;
		for (int j = 0; j < pstMatch->nUavEnemyNum; j++)
		{
			if (pstMatch->astEnemyUav[j].nX == pstMatch->astGoods[i_good].nStartX&& pstMatch->astEnemyUav[j].nY == pstMatch->astGoods[i_good].nStartY)
			{
				for (int k = 0; k < pstFlayPlane->nUavNum; k++)
				{
					if ((pstFlayPlane->astUav[k].task == 7 && pstFlayPlane->astUav[k].AttackUavNo == pstMatch->astEnemyUav[j].nNO&&pstFlayPlane->astUav[k].nStatus != UAV_CRASH) ||
						(pstFlayPlane->astUav[k].task == 4 && pstFlayPlane->astUav[k].AttackUavNo == pstMatch->astEnemyUav[j].nNO&&pstFlayPlane->astUav[k].nStatus != UAV_CRASH))
					{
						cantGet = true;
						break;
					}
				}
				break;
			}
		}
		if (cantGet == true)
			continue;
		AStar::Vec3i goodPoint = { pstMatch->astGoods[i_good].nStartX, pstMatch->astGoods[i_good].nStartY, 0 };
		AStar::Vec3i endPoint = { pstMatch->astGoods[i_good].nEndX, pstMatch->astGoods[i_good].nEndY, 0 };
		int steps2 = EstimateSteps(goodPoint, endPoint,pstMap);
		std::vector<COST_PERF> tempTable;
		std::vector<int> steps1Vec;
		for (int i_uav = 0; i_uav < pstFlayPlane->nUavNum; i_uav++)
		{
			if (pstFlayPlane->astUav[i_uav].nStatus == UAV_CRASH)
				continue;
			if (pstFlayPlane->astUav[i_uav].nGoodsNo != -1)
				continue;
			if (pstFlayPlane->astUav[i_uav].task == 2 || pstFlayPlane->astUav[i_uav].task == 4 || pstFlayPlane->astUav[i_uav].task == 5 || pstFlayPlane->astUav[i_uav].task == 6 || pstFlayPlane->astUav[i_uav].task == 7 || pstFlayPlane->astUav[i_uav].task == 8)
				continue;
			/*if (pstFlayPlane->astUav[i_uav].szType[1]==cheapestUav.szType[1])
			continue;*/
			if (pstFlayPlane->astUav[i_uav].nLoadWeight < pstMatch->astGoods[i_good].nWeight)
				continue;
			AStar::Vec3i startPoint = { pstFlayPlane->astUav[i_uav].nX, pstFlayPlane->astUav[i_uav].nY, pstFlayPlane->astUav[i_uav].nZ };
			int steps1 = EstimateSteps(startPoint, goodPoint, pstMap);
			int remainnow = pstMatch->astGoods[i_good].nRemainTime + pstMatch->astGoods[i_good].nStartTime - pstMatch->nTime;
			if (steps1>remainnow)
					continue;
			//�жϵ���
			int needELec = pstMatch->astGoods[i_good].nWeight*(steps2);
			if (needELec>pstFlayPlane->astUav[i_uav].remain_electricity*0.8)
				continue;
			float CostPerformance = (float)pstMatch->astGoods[i_good].nValue /((steps1 + steps2));
			//float CostPerformance = (float)1 / (steps1 + steps2);
			COST_PERF temp;
			temp.uavIndex =i_uav;
			temp.goodsIndex = i_good;
			temp.costPerformence = CostPerformance;
			steps1Vec.push_back(steps1);
			tempTable.push_back(temp);
				
		}
		//int aMinstep1 = 500;
		//for (int ii = 0; ii <steps1Vec.size(); ii++)
		//{
		//	if (aMinstep1 > steps1Vec[ii])
		//		aMinstep1 = steps1Vec[ii];
		//}
		////�з��ɻ�
		//bool Issave = true;
		//int enemyMinSteps=500;
		//for (int i_uav = 0; i_uav < pstMatch->nUavEnemyNum; i_uav++)
		//{
		//	if (pstMatch->astEnemyUav[i_uav].nStatus == UAV_CRASH)
		//		continue;
		//	if (pstMatch->astEnemyUav[i_uav].nGoodsNo != -1)
		//		continue;
		//	if (pstMatch->astEnemyUav[i_uav].nLoadWeight < pstMatch->astGoods[i_good].nWeight)
		//		continue;
		//		AStar::Vec3i startPoint = { pstMatch->astEnemyUav[i_uav].nX, pstMatch->astEnemyUav[i_uav].nY, pstMatch->astEnemyUav[i_uav].nZ };
		//		int steps3 = EstimateSteps(startPoint, goodPoint,pstMap);
		//		if (steps3 < enemyMinSteps)
		//			enemyMinSteps = steps3;
		//}

		//if (enemyMinSteps<aMinstep1)
		//	continue;
		//else
		costPerfTable.push_back(tempTable);

	}

	//�Ѿ�ȡ����Ʒ�ķɻ�
	if (pstMatch->nTime == 135)
	{
		int ll = 0;
	}
	//uavNumNoGoods
	int uavNumNoGoods = 0;
	for (int i_uav = 0; i_uav < pstMatch->nUavWeNum; i_uav++)
	{
		if (pstFlayPlane->astUav[i_uav].nStatus == UAV_CRASH)
			continue;
		if (pstFlayPlane->astUav[i_uav].nGoodsNo != -1)
			continue;
		if (pstFlayPlane->astUav[i_uav].task == 2 || pstFlayPlane->astUav[i_uav].task == 4 || pstFlayPlane->astUav[i_uav].task == 5 || pstFlayPlane->astUav[i_uav].task == 6 || pstFlayPlane->astUav[i_uav].task == 7 || pstFlayPlane->astUav[i_uav].task == 8)
			continue;
		uavNumNoGoods++;
	}
	
	for (int i_times = 0; i_times <uavNumNoGoods; i_times++)//ѭ�����˻�������
	{
		int theMaxi = 0;
		int theMaxj = 0;
		float maxCostPerformence = 0;
		for (int i_tableGood = 0; i_tableGood <costPerfTable.size(); i_tableGood++)
		{
			for (int i_table = 0; i_table < costPerfTable[i_tableGood].size(); i_table++)
			{
				if (costPerfTable[i_tableGood][i_table].costPerformence > maxCostPerformence)
				{
					maxCostPerformence = costPerfTable[i_tableGood][i_table].costPerformence;
					theMaxi = i_tableGood;
					theMaxj = i_table;
				}
				else if (costPerfTable[i_tableGood][i_table].costPerformence == maxCostPerformence)
				{
					if (pstFlayPlane->astUav[costPerfTable[i_tableGood][i_table].uavIndex].nLoadWeight<pstFlayPlane->astUav[theMaxi].nLoadWeight)
					{
						maxCostPerformence = costPerfTable[i_tableGood][i_table].costPerformence;
						theMaxi = i_tableGood;
						theMaxj = i_table;
					}
				}
			}
		}
		if (maxCostPerformence == 0)
			break;
		//�Ա����һ������
		for (int i_tableGood = 0; i_tableGood <costPerfTable.size(); i_tableGood++)
		{
			for (int i_table = 0; i_table < costPerfTable[i_tableGood].size(); i_table++)
			{
				if (costPerfTable[i_tableGood][i_table].uavIndex == costPerfTable[theMaxi][theMaxj].uavIndex || costPerfTable[i_tableGood][i_table].goodsIndex == costPerfTable[theMaxi][theMaxj].goodsIndex)
				{
					costPerfTable[i_tableGood][i_table].costPerformence = 0;
				}
			}
		}
		
		AStar::Vec3i startPoint = { pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].nX, pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].nY, pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].nZ };
		AStar::Vec3i goodPointh = { pstMatch->astGoods[costPerfTable[theMaxi][theMaxj].goodsIndex].nStartX, pstMatch->astGoods[costPerfTable[theMaxi][theMaxj].goodsIndex].nStartY, pstMap->nHLow };
		AStar::CoordinateList  tempList1;
		MinStepsCal(startPoint, goodPointh, &tempList1);//�������ͻ�������·��
		if (startPoint.x != goodPointh.x || startPoint.y != goodPointh.y)
		{
			//movetoGoods
			move2point(pstMatch, pstFlayPlane, tempList1, &pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex]);
			pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].task = 1;
			pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].targetGood = pstMatch->astGoods[costPerfTable[theMaxi][theMaxj].goodsIndex];
			pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].haveDeal = true;
		}
		else
		{
			AStar::CoordinateList  tempList2;
			AStar::Vec3i goodPoint = { pstMatch->astGoods[costPerfTable[theMaxi][theMaxj].goodsIndex].nStartX, pstMatch->astGoods[costPerfTable[theMaxi][theMaxj].goodsIndex].nStartY, 0 };
			MinStepsCal(startPoint, goodPoint, &tempList2);//�������ͻ�������·��
			AStar::CoordinateList  tempList3;
			AStar::Vec3i endPoint = { pstMatch->astGoods[costPerfTable[theMaxi][theMaxj].goodsIndex].nEndX, pstMatch->astGoods[costPerfTable[theMaxi][theMaxj].goodsIndex].nEndY, 0 };
			MinStepsCal(goodPoint, endPoint, &tempList3);//����������ͻ����յ�����·��
			pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].path = tempList2;
			pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].pathNext = tempList3;
			pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].task = 6;
			pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].targetGood = pstMatch->astGoods[costPerfTable[theMaxi][theMaxj].goodsIndex];
			GetGood(pstMatch, pstFlayPlane, &pstMatch->astGoods[costPerfTable[theMaxi][theMaxj].goodsIndex], &pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex], tempList2, tempList3);
			pstFlayPlane->astUav[costPerfTable[theMaxi][theMaxj].uavIndex].haveDeal = true;
		}
	



		
	}
	//clock_t start, finish;
	//double totaltime;
	//start = clock();
	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//std::cout << "\n�˳��������ʱ��Ϊ" << totaltime << "�룡" << std::endl;

	if (pstMatch->nTime == 143)
	{
		int ll = 0;
	}

	srand((unsigned)time(NULL));
	for (int i = 0; i < pstFlayPlane->nUavNum; i++)
	{
		if (pstFlayPlane->astUav[i].nStatus == UAV_CRASH)
			continue;
		if (pstFlayPlane->astUav[i].task == 1 || pstFlayPlane->astUav[i].task == 2 || pstFlayPlane->astUav[i].task == 4 || pstFlayPlane->astUav[i].task == 5 || pstFlayPlane->astUav[i].task == 6 || pstFlayPlane->astUav[i].task == 7 || pstFlayPlane->astUav[i].task == 8)
			continue;
	/*	if (pstFlayPlane->astUav[i].szType[1] != cheapestUav.szType[1])
			continue;*/
	
		//ȥ����ɻ���
		int ran= rand();
		SpreadOut(pstMap, pstMatch, pstFlayPlane, &pstFlayPlane->astUav[i],ran);
		pstFlayPlane->astUav[i].haveDeal = true;
		//toEnemyParking(pstMap, pstMatch, pstFlayPlane, &pstFlayPlane->astUav[i]);
	
	}

	int smallUavNum = 0;
	for (int i = 0; i < pstMatch->nUavWeNum; i++)
	{
		if (pstMatch->astWeUav[i].nStatus == UAV_CRASH)
			continue;
		if (pstMatch->astWeUav[i].szType[1] == cheapestUav.szType[1])
			smallUavNum++;
	}
	

	
	//����С�ɻ�
	if (pstMatch->nWeValue>cheapestUav.nValue)
	{
		pstFlayPlane->nPurchaseNum = 1;
		pstFlayPlane->szPurchaseType[0][0] = 'F';
		pstFlayPlane->szPurchaseType[0][1] = cheapestUav.szType[1];
	}
	//���������

	













}


int main(int argc, char *argv[])
{
#ifdef OS_WINDOWS
	// windows�£���Ҫ���г�ʼ������
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("WSAStartup failed\n");
		return false;
	}

#endif

	char        szIp[64] = { 0 };
	int         nPort = 0;
	char        szToken[128] = { 0 };
	int         nRet = 0;
	OS_SOCKET   hSocket;
	char        *pRecvBuffer = NULL;
	char        *pSendBuffer = NULL;
	int         nLen = 0;

	// ���ص���ȥ�����
	if (argc != 4)
	{
		printf("error arg num\n");
		return -1;
	}

	// ��������
	strcpy(szIp, argv[1]);
	nPort = atoi(argv[2]);
	strcpy(szToken, argv[3]);

	//     strcpy(szIp, "39.106.111.130");
	//     nPort = 4010;
	//     strcpy(szToken, "36d0a20b-7fab-4a93-b7e4-3247533b903a");

	printf("server ip %s, prot %d, token %s\n", szIp, nPort, szToken);

	// ��ʼ���ӷ�����
	nRet = OSCreateSocket(szIp, (unsigned int)nPort, &hSocket);
	if (nRet != 0)
	{
		printf("connect server error\n");
		return nRet;
	}

	// ������ܷ����ڴ�
	pRecvBuffer = (char*)malloc(MAX_SOCKET_BUFFER);
	if (pRecvBuffer == NULL)
	{
		return -1;
	}

	pSendBuffer = (char*)malloc(MAX_SOCKET_BUFFER);
	if (pSendBuffer == NULL)
	{
		free(pRecvBuffer);
		return -1;
	}

	memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);

	// ��������  ���ӳɹ���Judger�᷵��һ����Ϣ��
	nRet = RecvJuderData(hSocket, pRecvBuffer);
	if (nRet != 0)
	{
		return nRet;
	}

	// json ����
	// ��ȡͷ��
	CONNECT_NOTICE  stNotice;

	nRet = ParserConnect(pRecvBuffer + SOCKET_HEAD_LEN, &stNotice);
	if (nRet != 0)
	{
		return nRet;
	}

	// ���ɱ�����ݵ�json
	TOKEN_INFO  stToken;

	strcpy(stToken.szToken, szToken);  // ����ǵ��Խ׶Σ�����������Ե�token�����ҵĶ�ս�л�ȡ��
	// ʵ�ʱ�������Ҫ������Եģ�����demoд�ģ��з��������ô��롣
	strcpy(stToken.szAction, "sendtoken");

	memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);//pSendBuffer����ʱ�յģ�ȫ��ʱ0
	nRet = CreateTokenInfo(&stToken, pSendBuffer, &nLen);
	/*
	00000076{
	"token":	"b44601bd-64e0-474e-bcd8-65016a9e5876",
	"action":	"sendtoken"
	}*/
	if (nRet != 0)
	{
		return nRet;
	}
	// ѡ������з������������(Player -> Judger)
	nRet = SendJuderData(hSocket, pSendBuffer, nLen);
	if (nRet != 0)
	{
		return nRet;
	}

	//�����֤���(Judger -> Player)��
	memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = RecvJuderData(hSocket, pRecvBuffer);
	if (nRet != 0)
	{
		return nRet;
	}



	// ������֤�����json
	TOKEN_RESULT      stResult;
	nRet = ParserTokenResult(pRecvBuffer + SOCKET_HEAD_LEN, &stResult);
	if (nRet != 0)
	{
		return 0;
	}

	// �Ƿ���֤�ɹ�
	if (stResult.nResult != 0)
	{
		printf("token check error\n");
		return -1;
	}
	// ѡ������з����������Լ���׼������(Player -> Judger)
	READY_PARAM     stReady;

	strcpy(stReady.szToken, szToken);
	strcpy(stReady.szAction, "ready");

	memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = CreateReadyParam(&stReady, pSendBuffer, &nLen);
	if (nRet != 0)
	{
		return nRet;
	}
	nRet = SendJuderData(hSocket, pSendBuffer, nLen);
	if (nRet != 0)
	{
		return nRet;
	}

	//��ս��ʼ֪ͨ(Judger -> Player)�� 
	memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
	nRet = RecvJuderData(hSocket, pRecvBuffer);
	if (nRet != 0)
	{
		return nRet;
	}
	// ��������
	//Mapinfo �ṹ����ܴܺ󣬲�̫�ʺϷ���ջ�У����Զ���Ϊȫ�ֻ����ڴ����
	MAP_INFO            *pstMapInfo;
	MATCH_STATUS        *pstMatchStatus;
	FLAY_PLANE          *pstFlayPlane;

	pstMapInfo = (MAP_INFO *)malloc(sizeof(MAP_INFO));//��̬�ֱ����ڴ�
	if (pstMapInfo == NULL)
	{
		return -1;
	}

	pstMatchStatus = (MATCH_STATUS *)malloc(sizeof(MATCH_STATUS));
	if (pstMapInfo == NULL)
	{
		return -1;
	}

	pstFlayPlane = (FLAY_PLANE *)malloc(sizeof(FLAY_PLANE));
	if (pstFlayPlane == NULL)
	{
		return -1;
	}

	memset(pstMapInfo, 0, sizeof(MAP_INFO));//ȫ�����Ϊ0
	memset(pstMatchStatus, 0, sizeof(MATCH_STATUS));
	memset(pstFlayPlane, 0, sizeof(FLAY_PLANE));

	nRet = ParserMapInfo(pRecvBuffer + SOCKET_HEAD_LEN, pstMapInfo);
	if (nRet != 0)
	{
		return nRet;
	}

	// ��һ�ΰ����˻��ĳ�ʼ��ֵ��flayplane
	pstFlayPlane->nPurchaseNum = 0;
	pstFlayPlane->nUavNum = pstMapInfo->nUavNum;
	for (int i = 0; i < pstMapInfo->nUavNum; i++)
	{
		pstFlayPlane->astUav[i] = pstMapInfo->astUav[i];
	}
	//��һЩ��������Ϊȫ�ֵģ����������������λ������ʼ��


	//InitRegionTable(pstMapInfo, pstMatchStatus);
	InitUavInfo(pstMapInfo);
	// ���ݷ�����ָ���ͣ�Ľ��ܷ�������
	while (1)
	{
		if (pstMatchStatus->nTime == 1)
		{
			SetMap(pstMapInfo, pstMatchStatus);
		}
		// ���е�ǰʱ�̵����ݼ���, �����мƻ��ṹ�壬ע�⣺0ʱ�̲��ܽ����ƶ�������һ�ν����ѭ��ʱ
		if (pstMatchStatus->nTime != 0)
		{
			AlgorithmCalculationFun(pstMapInfo, pstMatchStatus, pstFlayPlane);//ͨ�� AlgorithmCalculationFunҪ�ı�pstFlayPlane
		}


		//���ͷ��мƻ��ṹ��
		memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
		nRet = CreateFlayPlane(pstFlayPlane, szToken, pSendBuffer, &nLen);
		if (nRet != 0)
		{
			return nRet;
		}
		nRet = SendJuderData(hSocket, pSendBuffer, nLen);
		if (nRet != 0)
		{
			return nRet;
		}

		printf("%s\n", pSendBuffer);

		// ���ܵ�ǰ����״̬
		memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
		nRet = RecvJuderData(hSocket, pRecvBuffer);
		if (nRet != 0)
		{
			return nRet;
		}

		// ����
		nRet = ParserMatchStatus(pRecvBuffer + SOCKET_HEAD_LEN, pstMatchStatus);//ͨ��pRecvBuffer������pstMatchStatus��״̬
		if (nRet != 0)
		{
			return nRet;
		}

		if (pstMatchStatus->nMacthStatus == 1)
		{
			// ��������
			printf("game over, we value %d, enemy value %d\n", pstMatchStatus->nWeValue, pstMatchStatus->nEnemyValue);
			return 0;
		}
	}

	// �ر�socket
	OSCloseSocket(hSocket);
	// ��Դ����
	free(pRecvBuffer);
	free(pSendBuffer);
	free(pstMapInfo);
	free(pstMatchStatus);
	free(pstFlayPlane);

	return 0;
}