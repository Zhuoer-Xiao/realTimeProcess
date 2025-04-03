#pragma once
#pragma once
#include <QtWidgets/qinputdialog.h>
#include <QtWidgets/qformlayout.h>
#include <QtWidgets/QMainWindow>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>s
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include "ui_realTimeProcess.h"
#include <QMessageBox>
#include <pcl/visualization/pcl_visualizer.h>
#include <QFileDialog>
#include "realSystem.h"
#include "logger.h"
#include <boost/thread/thread.hpp>
#include "registrations.h"
#include "postProcess.h"
#include "dialogs.h"
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class realTimeProcess : public QMainWindow {
	Q_OBJECT

public:
	realTimeProcess(QWidget* parent = nullptr);
	~realTimeProcess();

private:
	Ui::realTimeProcessClass ui;

	PointCloudT::Ptr cloudShow;
	PointCloudT::Ptr cloudTar;
	PointCloudT::Ptr cloudSrc;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view;
	//��ر�־λ
    bool isLoadSrc = false;
    bool isLoadTar = false;
	//�༭��ǣ�0ΪԴ���ƣ�1ΪĿ�����
    int editFlag = 0;


	//��־
	Logger logger;

	void initialVtkWidget();

private slots:
	//todo:���ص����ļ�
	void loadSrc();
	void loadTar();
	//todo:����ʵʱƴ��ϵͳ
	void enterSystem();
	//todo:��ʼƴ��
	void startReg();
	//todo:��ʼ����
	void startPostProcess();
	//todo:�������
	void clearPointCloud();
	//todo:������ɫ�趨
    void setBackgroundColor();
	//todo:���浱ǰ����
	void saveCurrentPointCloud();
	//todo:errorInfo
    void errorInfo(int errNum);
	//todo:������ʾ����
	void updatePointCloudShow();

	//
	void editSrc();
	void editTar();

	//�ϲ����Ƶ�Ŀ�����
    void mergeCloud();
};

