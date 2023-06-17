#include "IppFeature.h"
#include <cmath>
#include <vector>
#include "IppFilter.h"

// 마스크 기반 엣지 검출 함수(로버츠)
void IppEdgeRoberts(IppByteImage& img, IppByteImage& imgEdge)
{
	int w = img.GetWidth();
	int h = img.GetHeight();

	imgEdge.CreateImage(w, h);

	BYTE** p1 = img.GetPixels2D();
	BYTE** p2 = imgEdge.GetPixels2D();

	int i, j, h1, h2;
	double hval;
	for (j = 1; j < h - 1; j++)
		for (i = 1; i < w - 1; i++)
		{
			// 관심 픽셀에 대해 마스크 연산(대각선)
			h1 = p1[j][i] - p1[j - 1][i - 1];
			h2 = p1[j][i] - p1[j - 1][i + 1];

			// |그래디언트 f| 구하기(루트 변화량^2 + 변화량^2))
			hval = sqrt(static_cast<double>(h1 * h1 + h2 * h2));

			p2[j][i] = static_cast<BYTE>(limit(hval + 0.5));
		}
}

// 마스크 기반 엣지 검출 함수(프리윗)
void IppEdgePrewitt(IppByteImage& img, IppByteImage& imgEdge)
{
	int w = img.GetWidth();
	int h = img.GetHeight();

	imgEdge.CreateImage(w, h);

	BYTE** p1 = img.GetPixels2D();
	BYTE** p2 = imgEdge.GetPixels2D();

	int i, j, h1, h2;
	double hval;
	for (j = 1; j < h - 1; j++)
		for (i = 1; i < w - 1; i++)
		{	
			// 관심 픽셀에 대해 마스크 연산
			h1 = -p1[j - 1][i - 1] - p1[j - 1][i] - p1[j - 1][i + 1]
				+ p1[j + 1][i - 1] + p1[j + 1][i] + p1[j + 1][i + 1];
			h2 = -p1[j - 1][i - 1] - p1[j][i - 1] - p1[j + 1][i - 1]
				+ p1[j - 1][i + 1] + p1[j][i + 1] + p1[j + 1][i + 1];

			// |그래디언트 f| 구하기(루트 변화량^2 + 변화량^2))
			hval = sqrt(static_cast<double>(h1 * h1 + h2 * h2));
			p2[j][i] = static_cast<BYTE>(limit(hval + 0.5));
		}
}

// 마스크 기반 엣지 검출 함수(소벨)
void IppEdgeSobel(IppByteImage& img, IppByteImage& imgEdge)
{
	int w = img.GetWidth();
	int h = img.GetHeight();

	imgEdge.CreateImage(w, h);

	BYTE** p1 = img.GetPixels2D();
	BYTE** p2 = imgEdge.GetPixels2D();

	int i, j, h1, h2;
	double hval;
	for (j = 1; j < h - 1; j++)
		for (i = 1; i < w - 1; i++)
		{
			// 관심 픽셀에 대해 마스크 연산
			h1 = -p1[j - 1][i - 1] - 2 * p1[j - 1][i] - p1[j - 1][i + 1]
				+ p1[j + 1][i - 1] + 2 * p1[j + 1][i] + p1[j + 1][i + 1];
			h2 = -p1[j - 1][i - 1] - 2 * p1[j][i - 1] - p1[j + 1][i - 1]
				+ p1[j - 1][i + 1] + 2 * p1[j][i + 1] + p1[j + 1][i + 1];

			// |그래디언트 f| 구하기(루트 변화량^2 + 변화량^2))
			hval = sqrt(static_cast<double>(h1 * h1 + h2 * h2));
			p2[j][i] = static_cast<BYTE>(limit(hval + 0.5));
		}
}

// 캐니 엣지 검출기

// 파이 값
const double PI = 3.14159265358979323846;
const float PI_F = 3.14159265358979323846f;

#define CHECK_WEAK_EDGE(x, y) \
	if (pEdge[y][x] == WEAK_EDGE) { \
	pEdge[y][x] = STRONG_EDGE; \
	strong_edges.push_back(IppPoint(x, y)); \
	}   // push_back() 값을 벡터의 뒤에 추가

// 케니 엣지 검출기
void IppEdgeCanny(IppByteImage& imgSrc, IppByteImage& imgEdge, float sigma, float th_low, float th_high)
{
	register int i, j;

	int w = imgSrc.GetWidth();
	int h = imgSrc.GetHeight();

	// 1. 가우시안 필터링
	IppFloatImage imgGaussian(w, h);
	IppFilterGaussian(imgSrc, imgGaussian, sigma);

	// 2. 그래디언트 구하기 (크기)
	IppFloatImage imgGx(w, h); // gradient of x, 수평 방향 소벨 마스크 연산 수행 결과
	IppFloatImage imgGy(w, h); // gradient of y, 수직 방향 소벨 마스크 연산 수행 결과
	IppFloatImage imgMag(w, h); // magnitude of gradient, 그래디언트 크기 저장

	float** pGauss = imgGaussian.GetPixels2D(); // 가우시안 필터링 결과
	float** pGx = imgGx.GetPixels2D();   // 그래디언트 방향(수평 방향 소벨 마스크)
	float** pGy = imgGy.GetPixels2D();	 // 그래디언트 방향(수직 방향 소벨 마스크)
	float** pMag = imgMag.GetPixels2D(); // 그래디언트 크기

	for (j = 1; j < h - 1; j++)
		for (i = 1; i < w - 1; i++)
		{
			pGx[j][i] = -pGauss[j - 1][i - 1] - 2 * pGauss[j][i - 1] - pGauss[j + 1][i - 1] + pGauss[j - 1][i + 1] + 2 * pGauss[j][i + 1] + pGauss[j + 1][i + 1];
			pGy[j][i] = -pGauss[j - 1][i - 1] - 2 * pGauss[j - 1][i] - pGauss[j - 1][i + 1] + pGauss[j + 1][i - 1] + 2 * pGauss[j + 1][i] + pGauss[j + 1][i + 1];
			pMag[j][i] = sqrt(pGx[j][i] * pGx[j][i] + pGy[j][i] * pGy[j][i]);
		}

	// 3. 비최대 억제
	// 국지적 최대를 구함과 동시에 이중 임계값(th_high, th_low)을 적용하여 strong edge와 weak edge를 구한다.
	imgEdge.CreateImage(w, h);
	BYTE** pEdge = imgEdge.GetPixels2D();

	// 구역? 0, 45, 90, 135, ??
	enum DISTRICT { AREA0 = 0, AREA45, AREA90, AREA135, NOAREA };

	// 강한 엣지, 약한 엣지 값 지정
	const BYTE STRONG_EDGE = 255;
	const BYTE WEAK_EDGE = 128;

	// 강한 엣지 픽셀 위치 저장 벡터
	std::vector<IppPoint> strong_edges;

	float ang; // 각도인가..
	int district; // 구역?
	bool local_max;  // 국지적 최대 여부?
	for (j = 1; j < h - 1; j++)
		for (i = 1; i < w - 1; i++)
		{
			// 그래디언트 크기가 th_low보다 큰 픽셀에 대해서만 국지적 최대 검사.
			// 국지적 최대인 픽셀에 대해서만 강한 엣지 또는 약한 엣지로 설정.
			if (pMag[j][i] > th_low)
			{
				// 그래디언트 방향 계산 (4개 구역)
				if (pGx[j][i] != 0.f)
				{
					ang = atan2(pGy[j][i], pGx[j][i]) * 180 / PI_F;
					if (((ang >= -22.5f) && (ang < 22.5f)) || (ang >= 157.5f) && (ang < -157.5f))
						district = AREA0;
					else if (((ang >= 22.5f) && (ang < 67.5f)) || ((ang >= -157.5f) && (ang < -112.5f)))
						district = AREA45;
					else if (((ang >= 67.5f) && (ang < 112.5f)) || ((ang >= -112.5f) && (ang < -67.5f)))
						district = AREA90;
					else
						district = AREA135;
				}
				else
					district = AREA90;

				// (구역에 따른) 국지적 최대 검사
				local_max = false;
				switch (district)
				{
				case AREA0:
					if ((pMag[j][i] >= pMag[j][i - 1]) && (pMag[j][i] > pMag[j][i + 1]))
						local_max = true;
					break;
				case AREA45:
					if ((pMag[j][i] >= pMag[j - 1][i - 1]) && (pMag[j][i] > pMag[j + 1][i + 1]))
						local_max = true;
					break;
				case AREA90:
					if ((pMag[j][i] >= pMag[j - 1][i]) && (pMag[j][i] > pMag[j + 1][i]))
						local_max = true;
					break;
				case AREA135:
				default:
					if ((pMag[j][i] >= pMag[j - 1][i + 1]) && (pMag[j][i] > pMag[j + 1][i - 1]))
						local_max = true;
					break;
				}

				// 강한 엣지와 약한 엣지 구분.
				if (local_max)  // 국지적 최대 유무
				{
					if (pMag[j][i] > th_high)  // 임계값보다 큰지, 작은지 여부에 따라
					{
						pEdge[j][i] = STRONG_EDGE;
						strong_edges.push_back(IppPoint(i, j));  // 강한 엣지 벡터에 추가
					}
					else
						pEdge[j][i] = WEAK_EDGE;
				}
			}
		}

		// 4. 히스테리시스 엣지 트래킹
		while (!strong_edges.empty())
		{
			IppPoint p = strong_edges.back();  // 벡터의 마지막 요소 반환(삭제 x)
			strong_edges.pop_back();  // 맨 뒤에 요소 삭제

			int x = p.x, y = p.y;

			// 강한 엣지 주변의 약한 엣지는 최종 엣지(강한 엣지)로 설정
			CHECK_WEAK_EDGE(x + 1, y)
			CHECK_WEAK_EDGE(x + 1, y + 1)
			CHECK_WEAK_EDGE(x, y + 1)
			CHECK_WEAK_EDGE(x - 1, y + 1)
			CHECK_WEAK_EDGE(x - 1, y)
			CHECK_WEAK_EDGE(x - 1, y - 1)
			CHECK_WEAK_EDGE(x, y - 1)
			CHECK_WEAK_EDGE(x + 1, y - 1)
		}

		// 모든 픽셀을 돌면서
		// 끝까지 약한 엣지로 남아있는 픽셀은 모두 엣지가 아닌 것으로 판단.
		for (j = 0; j < h; j++)
			for (i = 0; i < w; i++)
				if (pEdge[j][i] == WEAK_EDGE) pEdge[j][i] = 0;
}

// 룩업 테이블을 이용한 허프 변환의 구현
void IppHoughLine(IppByteImage& img, std::vector<IppLineParam>& lines, int threshold)
{   // img는 엣지 픽셀로 구성된 엣지 영상(엣지는 255, 그 외 0)

	register int i, j;

	int w = img.GetWidth();
	int h = img.GetHeight();

	BYTE** ptr = img.GetPixels2D();

	// 각각 p, theta를 최대값으로 설정
	int num_rho = static_cast<int>(sqrt((double)w * w + h * h) * 2);
	int num_ang = 360; 

	// 룩업테이블 : 0 ~ PI 각도에 해당하는 sin, cos 함수의 값을 저장
	float* sin_tbl = new float[num_ang];
	float* cos_tbl = new float[num_ang];

	for (i = 0; i < num_ang; i++)
	{
		sin_tbl[i] = sin(i * PI_F / num_ang);
		cos_tbl[i] = cos(i * PI_F / num_ang);
	}

	// 축적 배열(Accumulate array) 생성
	IppIntImage imgAcc(num_ang, num_rho);
	int** pAcc = imgAcc.GetPixels2D();

	int m, n;
	for (j = 0; j < h; j++) {
		for (i = 0; i < w; i++) {
			if (ptr[j][i] > 128) { // 입력영상에서 128보다 큰값을 가진 부분을 엣지로 인식
				// 이게 뭐냐...
				for (n = 0; n < num_ang; n++) {
					m = static_cast<int>(floor(i * sin_tbl[n] + j * cos_tbl[n]));
					m += (num_rho / 2);
					pAcc[m][n]++;
				}
			}
		}
	}//for+for

	// 임계값보다 큰 국지적 최대값을 찾아 직선 성분으로 결정
	lines.clear();
	int value;
	for (m = 0; m < num_rho; m++) {
		for (n = 0; n < num_ang; n++) {
			value = pAcc[m][n];
			// 임계값보다 큰
			if (value > threshold) {  
				// 국지적 최대
				if (value >= pAcc[m - 1][n] && value >= pAcc[m - 1][n + 1] &&
					value >= pAcc[m][n + 1] && value >= pAcc[m + 1][n + 1] &&
					value >= pAcc[m + 1][n] && value >= pAcc[m + 1][n - 1] &&
					value >= pAcc[m][n - 1] && value >= pAcc[m - 1][n - 1]) 
				{
					lines.push_back(IppLineParam(m - (num_rho / 2), n * PI / num_ang, pAcc[m][n]));
				}
			}
		}
	} //for+for

   // 동적 할당 메모리 해제
	delete[] sin_tbl;
	delete[] cos_tbl;
}

// 검출된 직선 그리기 함수
// c는 직선의 밝기
void IppDrawLine(IppByteImage& img, IppLineParam line, BYTE c)
{   
	int w = img.GetWidth();
	int h = img.GetHeight();

	// (rho, ang) 파라미터를 이용하여 직선의 시작 좌표와 끝 좌표를 계산

	int x1, y1, x2, y2;
	if ((line.ang >= 0 && line.ang < PI / 4) || (line.ang >= 3 * PI / 4 && line.ang < PI))
	{
		x1 = 0;
		y1 = static_cast<int>(floor(line.rho / cos(line.ang) + 0.5));
		x2 = w - 1;
		y2 = static_cast<int>(floor((line.rho - x2 * sin(line.ang)) / cos(line.ang) + 0.5));
	}
	else
	{
		y1 = 0;
		x1 = static_cast<int>(floor(line.rho / sin(line.ang) + 0.5));
		y2 = h - 1;
		x2 = static_cast<int>(floor((line.rho - y2 * cos(line.ang)) / sin(line.ang) + 0.5));
	}

	// x, y 값에 따라 직선을 그린다.
	IppDrawLine(img, x1, y1, x2, y2, c);
}

// x, y 값에 따라 직선을 그림. (내부적으로 호출되는 함수)
void IppDrawLine(IppByteImage& img, int x1, int y1, int x2, int y2, BYTE c)
{
	int w = img.GetWidth();
	int h = img.GetHeight();
	BYTE** ptr = img.GetPixels2D();

	// 브레제남 알고리즘(Bresenham's Algorithm)에 의한 직선 그리기

	int dx, dy, inc_x, inc_y, fraction;

	dx = x2 - x1;
	inc_x = (dx > 0) ? 1 : -1;
	dx = abs(dx) << 1;

	dy = y2 - y1;
	inc_y = (dy > 0) ? 1 : -1;
	dy = abs(dy) << 1;

	if (x1 >= 0 && x1 < w && y1 >= 0 && y1 < h)
		ptr[y1][x1] = c;

	if (dx >= dy)
	{
		fraction = dy - (dx >> 1);

		while (x1 != x2)
		{
			if ((fraction >= 0) && (fraction || (inc_x > 0)))
			{
				fraction -= dx;
				y1 += inc_y;
			}

			fraction += dy;
			x1 += inc_x;

			if (x1 >= 0 && x1 < w && y1 >= 0 && y1 < h)
				ptr[y1][x1] = c;
		}//while
	}//if
	else
	{
		fraction = dx - (dy >> 1);
		while (y1 != y2)
		{
			if ((fraction >= 0) && (fraction || (inc_y > 0)))
			{
				fraction -= dy;
				x1 += inc_x;
			}
			fraction += dx;
			y1 += inc_y;

			if (x1 >= 0 && x1 < w && y1 >= 0 && y1 < h)
				ptr[y1][x1] = c;
		}//while
	}//else
}//함수끝

// 해리스 코너 검출 함수
void IppHarrisCorner(IppByteImage& img, std::vector<IppPoint>& corners,	double th)
{   // 입력 영상, 검출된 코너 포인트 정보, 임계값

	register int i, j, x, y;

	int w = img.GetWidth();
	int h = img.GetHeight();

	BYTE** ptr = img.GetPixels2D();

	// 1. (fx)*(fx), (fx)*(fy), (fy)*(fy) 계산
	// 행렬 M에 정의되어 있던 Ix^2, IxIy, Iy^2

	IppFloatImage imgDx2(w, h); // a
	IppFloatImage imgDy2(w, h); // d
	IppFloatImage imgDxy(w, h); // b, c

	float** dx2 = imgDx2.GetPixels2D();
	float** dy2 = imgDy2.GetPixels2D();
	float** dxy = imgDxy.GetPixels2D();

	// 
	float tx, ty;
	for (j = 1; j < h - 1; j++) {
		for (i = 1; i < w - 1; i++)
		{
			// 수평 방향 변화량
			tx = (ptr[j - 1][i + 1] + ptr[j][i + 1] + ptr[j + 1][i + 1]
				- ptr[j - 1][i - 1] - ptr[j][i - 1] - ptr[j + 1][i - 1]) / 6.f;
			// 수직 방향 변화량
			ty = (ptr[j + 1][i - 1] + ptr[j + 1][i] + ptr[j + 1][i + 1]
				- ptr[j - 1][i - 1] - ptr[j - 1][i] - ptr[j - 1][i + 1]) / 6.f;

			dx2[j][i] = tx * tx;
			dy2[j][i] = ty * ty;
			dxy[j][i] = tx * ty;
		}
	}

	// 2. 가우시안 필터링
	IppFloatImage imgGdx2(w, h);
	IppFloatImage imgGdy2(w, h);
	IppFloatImage imgGdxy(w, h);

	float** gdx2 = imgGdx2.GetPixels2D();
	float** gdy2 = imgGdy2.GetPixels2D();
	float** gdxy = imgGdxy.GetPixels2D();

	float g[5][5] = { 
		{ 1, 4, 6, 4, 1 },
		{ 4, 16, 24, 16, 4 },
		{ 6, 24, 36, 24, 6 },
		{ 4, 16, 24, 16, 4 },
		{ 1, 4, 6, 4, 1 } 
	};

	for (y = 0; y < 5; y++) {
		for (x = 0; x < 5; x++)
		{
			g[y][x] /= 256.f;
		}
	}

	float tx2, ty2, txy;
	for (j = 2; j < h - 2; j++) {
		for (i = 2; i < w - 2; i++)
		{
			tx2 = ty2 = txy = 0;
			for (y = 0; y < 5; y++)
				for (x = 0; x < 5; x++)
				{
					tx2 += (dx2[j + y - 2][i + x - 2] * g[y][x]);
					ty2 += (dy2[j + y - 2][i + x - 2] * g[y][x]);
					txy += (dxy[j + y - 2][i + x - 2] * g[y][x]);
				}

			gdx2[j][i] = tx2;
			gdy2[j][i] = ty2;
			gdxy[j][i] = txy;
		}
	}

	// 3. 코너 응답 함수 R 생성
	IppFloatImage imgCrf(w, h);
	float** crf = imgCrf.GetPixels2D();

	float k = 0.04f; // 상수 k
	for (j = 2; j < h - 2; j++) {
		for (i = 2; i < w - 2; i++)
		{	// R = Det(M) - k * Tr(M)
			// a = gdx2(Ix), b = c = gdxy(IxIy), d = gdy2(Iy)
			// Det(M) = ad - bc
			// Tr(M) = a + d
			// R = ad - bc - k * (a + d)
			crf[j][i] = (gdx2[j][i] * gdy2[j][i] - gdxy[j][i] * gdxy[j][i])  // Det(M)
				- k * (gdx2[j][i] + gdy2[j][i]) * (gdx2[j][i] + gdy2[j][i]); // Tr(M)
		}
	}

	// 4. 임계값보다 큰 국지적 최대값을 찾아 코너 포인트로 결정
	corners.clear();
	float cvf_value;
	for (j = 2; j < h - 2; j++) {
		for (i = 2; i < w - 2; i++)
		{
			cvf_value = crf[j][i];
			if (cvf_value > th)  // 임계값보다 큰
			{	// 국지적 최대(근처의 값과 비교)
				if (cvf_value > crf[j - 1][i] && cvf_value > crf[j - 1][i + 1] &&
					cvf_value > crf[j][i + 1] && cvf_value > crf[j + 1][i + 1] &&
					cvf_value > crf[j + 1][i] && cvf_value > crf[j + 1][i - 1] &&
					cvf_value > crf[j][i - 1] && cvf_value > crf[j - 1][i - 1]) {
					corners.push_back(IppPoint(i, j)); //코너포인트로 기록
				}
			}
		}//for
	}//for
}//함수끝
