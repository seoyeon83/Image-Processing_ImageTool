// CHistogramDlg.cpp: 구현 파일
//

#include "pch.h"
#include "ImageTool.h"
#include "CHistogramDlg.h"
#include "afxdialogex.h"


#include "IppDib.h"
#include "IppImage.h"
#include "IppConvert.h"
#include "IppImage/IppEnhance.h"


// CHistogramDlg 대화 상자

IMPLEMENT_DYNAMIC(CHistogramDlg, CDialogEx)

CHistogramDlg::CHistogramDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_HISTOGRAM, pParent)
{
	memset(m_Histogram, 0, sizeof(int) * 256);

}

CHistogramDlg::~CHistogramDlg()
{
}

void CHistogramDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

void CHistogramDlg::SetImage(IppDib* pDib)
{
	if (pDib != NULL && pDib->GetBitCount() == 8) {
		IppByteImage img;
		IppDibToImage(*pDib, img);

		float histo[256] = { 0.f };
		IppHistogram(img, histo);

		float max_histo = histo[0];
		for (int i = 0; i < 256; i++) {
			if (histo[i] > max_histo) max_histo = histo[i];
		}

		for (int i = 0; i < 256; i++) {
			m_Histogram[i] = static_cast<int>(histo[i] * 100 / max_histo);
		}
	}
	else {
		memset(m_Histogram, 0, sizeof(int) * 256);
	}
}


BEGIN_MESSAGE_MAP(CHistogramDlg, CDialogEx)
	ON_WM_PAINT()
END_MESSAGE_MAP()


// CHistogramDlg 메시지 처리기


void CHistogramDlg::OnPaint()
{
	CPaintDC dc(this); // device context for painting
					   // TODO: 여기에 메시지 처리기 코드를 추가합니다.
					   // 그리기 메시지에 대해서는 CDialogEx::OnPaint()을(를) 호출하지 마십시오.

	CGdiObject* pOldPen = dc.SelectStockObject(DC_PEN);

	dc.SetDCPenColor(RGB(128, 128, 128));
	dc.MoveTo(20, 20);
	dc.LineTo(20, 120);
	dc.LineTo(275, 120);
	dc.LineTo(275, 20);

	dc.SetDCPenColor(RGB(0, 0, 0));

	for (int i = 0; i < 256; i++) {
		dc.MoveTo(20 + i, 120);
		dc.LineTo(20 + i, 120 - m_Histogram[i]);
	}

	for (int i = 0; i < 256; i++) {
		dc.SetDCPenColor(RGB(i, i, i));
		dc.MoveTo(20 + i, 130);
		dc.LineTo(20 + i, 145);
	}

	dc.SelectObject(pOldPen);

}
