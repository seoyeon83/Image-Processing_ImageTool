#pragma once
#include "..\IppImage.h"
#include <vector>

// 마스크 기반 엣지 검출 함수(로버츠)
void IppEdgeRoberts(IppByteImage& img, IppByteImage& imgEdge);
// 마스크 기반 엣지 검출 함수(프리윗)
void IppEdgePrewitt(IppByteImage& img, IppByteImage& imgEdge);
// 마스크 기반 엣지 검출 함수(소벨)
void IppEdgeSobel(IppByteImage& img, IppByteImage& imgEdge);


// IppPoint 클래스: 픽셀 좌표 저장
class IppPoint
{
public:
	int x;
	int y;
public:
	IppPoint() : x(0), y(0) {}
	IppPoint(int _x, int _y) : x(_x), y(_y) {}
};

// 캐니 엣지 검출 함수
void IppEdgeCanny(IppByteImage& imgSrc, IppByteImage& imgEdge, float sigma, float th_low, float th_high);

// 직선 정의를 위한 파라미터 구조체
class IppLineParam
{
public:
	double rho;
	double ang;
	int vote; //축적배열에서 누적된 값
public:
	IppLineParam() :rho(0), ang(0), vote(0) {}
	IppLineParam(double r, double a, int v) :rho(r), ang(a), vote(v) {}
};

// 허프 변환을 이용한 직선 검출 함수
void IppHoughLine(IppByteImage& img, std::vector<IppLineParam>&lines, int threshold = 60);

// 검출된 직선 그리기 함수
void IppDrawLine(IppByteImage& img, IppLineParam line, BYTE c);
void IppDrawLine(IppByteImage& img, int x1, int y1, int x2, int y2, BYTE c);

// 허프변환 메뉴 이벤트처리기 함수
// STL의 sort함수사용을 위해 비교연산자 재정의
inline bool operator<(const IppLineParam& lhs, const IppLineParam& rhs) {
	return lhs.vote > rhs.vote;
}

// 해리스 코너 검출 함수
void IppHarrisCorner(IppByteImage& img, std::vector<IppPoint>& corners, double th);