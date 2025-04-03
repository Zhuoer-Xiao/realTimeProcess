#include <QApplication>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>  
#include <QDialogButtonBox>
#include <QPushButton>

//改进gicp输入框
class myGicpInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* scaleSpin1;
    QDoubleSpinBox* scaleSpin2;
    QDoubleSpinBox* scaleSpin3;
    QDoubleSpinBox* distanceSpin;
    QCheckBox* reciprocalCorrespondences;
    myGicpInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"改进GICP参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences = new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // 最大迭代次数（整数输入）
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // 允许1-1000
        maxIterationsSpin->setValue(100);        // 默认值

        // 第一尺度大小（浮点数输入）
        scaleSpin1 = new QDoubleSpinBox(this);
        scaleSpin1->setRange(0.1, 10.0);         // 允许0.1-10.0
        scaleSpin1->setValue(1.0);               // 默认值
        scaleSpin1->setSingleStep(0.1);          // 步长0.1

        // 第二尺度大小（浮点数输入）
        scaleSpin2 = new QDoubleSpinBox(this);
        scaleSpin2->setRange(0.1, 10.0);         // 允许0.1-10.0
        scaleSpin2->setValue(1.0);               // 默认值
        scaleSpin2->setSingleStep(0.1);          // 步长0.1

        // 第三尺度大小（浮点数输入）
        scaleSpin3 = new QDoubleSpinBox(this);
        scaleSpin3->setRange(0.1, 10.0);         // 允许0.1-10.0
        scaleSpin3->setValue(1.0);               // 默认值
        scaleSpin3->setSingleStep(0.1);          // 步长0.1

        // 最大对应点距离（浮点数输入）
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // 允许0-100
        distanceSpin->setValue(25.0);           // 默认值
        distanceSpin->setDecimals(2);           // 两位小数

        // 添加到表单布局
        formLayout->addRow(u8"最大迭代次数:", maxIterationsSpin);
        formLayout->addRow(u8"第一尺度大小:", scaleSpin1);
        formLayout->addRow(u8"第二尺度大小:", scaleSpin2);
        formLayout->addRow(u8"第三尺度大小:", scaleSpin3);
        formLayout->addRow(u8"最大对应点距离:", distanceSpin);
        formLayout->addRow(u8"点相互关系：", reciprocalCorrespondences);
        

        // 添加按钮
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // 连接信号槽
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // 获取输入值的公有方法
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getScale1() const { return scaleSpin1->value(); }
    double getScale2() const { return scaleSpin2->value(); }
    double getScale3() const { return scaleSpin3->value(); }
    double getMaxDistance() const { return distanceSpin->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
};


//gicp输入框
class gicpInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* scaleSpin;
    QDoubleSpinBox* distanceSpin;
    QCheckBox* reciprocalCorrespondences;
    gicpInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"GICP参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences= new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // 最大迭代次数（整数输入）
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // 允许1-1000
        maxIterationsSpin->setValue(100);        // 默认值

        // 尺度大小（浮点数输入）
        scaleSpin = new QDoubleSpinBox(this);
        scaleSpin->setRange(0.1, 10.0);         // 允许0.1-10.0
        scaleSpin->setValue(1.0);               // 默认值
        scaleSpin->setSingleStep(0.1);          // 步长0.1

        // 最大对应点距离（浮点数输入）
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // 允许0-100
        distanceSpin->setValue(25.0);           // 默认值
        distanceSpin->setDecimals(2);           // 两位小数

        // 添加到表单布局
        formLayout->addRow(u8"最大迭代次数:", maxIterationsSpin);
        formLayout->addRow(u8"尺度大小:", scaleSpin);
        formLayout->addRow(u8"最大对应点距离:", distanceSpin);
        formLayout->addRow(u8"点相互关系：", reciprocalCorrespondences);

        // 添加按钮
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // 连接信号槽
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // 获取输入值的公有方法
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getScale() const { return scaleSpin->value(); }
    double getMaxDistance() const { return distanceSpin->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
};

//icp输入框
class icpInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* scaleSpin;
    QDoubleSpinBox* distanceSpin;
    QCheckBox* reciprocalCorrespondences;
    icpInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"ICP参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences = new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // 最大迭代次数（整数输入）
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // 允许1-1000
        maxIterationsSpin->setValue(100);        // 默认值

        // 尺度大小（浮点数输入）
        scaleSpin = new QDoubleSpinBox(this);
        scaleSpin->setRange(0.1, 10.0);         // 允许0.1-10.0
        scaleSpin->setValue(1.0);               // 默认值
        scaleSpin->setSingleStep(0.1);          // 步长0.1

        // 最大对应点距离（浮点数输入）
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // 允许0-100
        distanceSpin->setValue(25.0);           // 默认值
        distanceSpin->setDecimals(2);           // 两位小数

        // 添加到表单布局
        formLayout->addRow(u8"最大迭代次数:", maxIterationsSpin);
        formLayout->addRow(u8"尺度大小:", scaleSpin);
        formLayout->addRow(u8"最大对应点距离:", distanceSpin);
        formLayout->addRow(u8"点相互关系：", reciprocalCorrespondences);

        // 添加按钮
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // 连接信号槽
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // 获取输入值的公有方法
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getScale() const { return scaleSpin->value(); }
    double getMaxDistance() const { return distanceSpin->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
};


class nicpInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* scaleSpin;
    QDoubleSpinBox* distanceSpin;
    QCheckBox* reciprocalCorrespondences;
    QSpinBox* neighborNum;
    nicpInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"NICP参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences = new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // 最大迭代次数（整数输入）
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // 允许1-1000
        maxIterationsSpin->setValue(100);        // 默认值

        // 尺度大小（浮点数输入）
        scaleSpin = new QDoubleSpinBox(this);
        scaleSpin->setRange(0.1, 10.0);         // 允许0.1-10.0
        scaleSpin->setValue(1.0);               // 默认值
        scaleSpin->setSingleStep(0.1);          // 步长0.1

        // 最大对应点距离（浮点数输入）
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // 允许0-100
        distanceSpin->setValue(25.0);           // 默认值
        distanceSpin->setDecimals(2);           // 两位小数

        //法向量邻居数
        neighborNum = new QSpinBox(this);
        neighborNum->setRange(5, 100);
        neighborNum->setValue(25);
        neighborNum->setSingleStep(1);

        // 添加到表单布局
        formLayout->addRow(u8"最大迭代次数:", maxIterationsSpin);
        formLayout->addRow(u8"尺度大小:", scaleSpin);
        formLayout->addRow(u8"最大对应点距离:", distanceSpin);
        formLayout->addRow(u8"法向量邻居数:", neighborNum);
        formLayout->addRow(u8"点相互关系：", reciprocalCorrespondences);

        // 添加按钮
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // 连接信号槽
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // 获取输入值的公有方法
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getScale() const { return scaleSpin->value(); }
    double getMaxDistance() const { return distanceSpin->value(); }
    int getNeighborNum() const { return neighborNum->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
};
//nonlinearIcp输入框
class nonlinearIcpInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* scaleSpin;
    QDoubleSpinBox* distanceSpin;
    QCheckBox* reciprocalCorrespondences;
    QSpinBox* neighborNum;
    nonlinearIcpInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"non-linear ICP参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences = new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // 最大迭代次数（整数输入）
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // 允许1-1000
        maxIterationsSpin->setValue(100);        // 默认值

        // 尺度大小（浮点数输入）
        scaleSpin = new QDoubleSpinBox(this);
        scaleSpin->setRange(0.1, 10.0);         // 允许0.1-10.0
        scaleSpin->setValue(1.0);               // 默认值
        scaleSpin->setSingleStep(0.1);          // 步长0.1

        // 最大对应点距离（浮点数输入）
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // 允许0-100
        distanceSpin->setValue(25.0);           // 默认值
        distanceSpin->setDecimals(2);           // 两位小数

        //法向量邻居数
        neighborNum = new QSpinBox(this);
        neighborNum->setRange(5, 100);
        neighborNum->setValue(25);
        neighborNum->setSingleStep(1);

        // 添加到表单布局
        formLayout->addRow(u8"最大迭代次数:", maxIterationsSpin);
        formLayout->addRow(u8"尺度大小:", scaleSpin);
        formLayout->addRow(u8"最大对应点距离:", distanceSpin);
        formLayout->addRow(u8"法向量邻居数:", neighborNum);
        formLayout->addRow(u8"点相互关系：", reciprocalCorrespondences);

        // 添加按钮
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // 连接信号槽
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // 获取输入值的公有方法
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getScale() const { return scaleSpin->value(); }
    double getMaxDistance() const { return distanceSpin->value(); }
    int getNeighborNum() const { return neighborNum->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
};

//NDT输入框:最大迭代次数、网格分辨率
class NDTInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* voxelSpin;
    NDTInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"NDT参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);

        // 最大迭代次数（整数输入）
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // 允许1-1000
        maxIterationsSpin->setValue(100);        // 默认值

        // 尺度大小（浮点数输入）
        voxelSpin = new QDoubleSpinBox(this);
        voxelSpin->setRange(0.1, 100.0);         // 允许0.1-10.0
        voxelSpin->setValue(1.0);               // 默认值
        voxelSpin->setSingleStep(0.1);          // 步长0.1

        

        // 添加到表单布局
        formLayout->addRow(u8"最大迭代次数:", maxIterationsSpin);
        formLayout->addRow(u8"网格分辨率:", voxelSpin);

        // 添加按钮
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // 连接信号槽
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // 获取输入值的公有方法
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getVoxel() const { return voxelSpin->value(); }
};

//fpcs参数设置：最大迭代次数；验证采样数量；近似重叠度
class fpcsInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QSpinBox* neighborNum;
    QDoubleSpinBox* overlapSpin;
    fpcsInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"FPCS参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        neighborNum = new QSpinBox(this);
        neighborNum->setRange(5, 100);
        neighborNum->setValue(25);
        neighborNum->setSingleStep(1);
        overlapSpin = new QDoubleSpinBox(this);
        overlapSpin->setRange(0.0, 1.0);
        overlapSpin->setValue(0.5);
        overlapSpin->setSingleStep(0.1);
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);
        maxIterationsSpin->setValue(100);
        formLayout->addRow(u8"最大迭代次数:", maxIterationsSpin);
        formLayout->addRow(u8"验证采样数量:", neighborNum);
        formLayout->addRow(u8"近似重叠度:", overlapSpin);
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
        formLayout->addRow(buttonBox);

        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    int getNeighborNum() const { return neighborNum->value(); }
    double getOverlap() const { return overlapSpin->value(); }
};

//k4pcs参数设置：最大迭代次数；验证采样数量；近似重叠度
class kfpcsInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QSpinBox* neighborNum;
    QDoubleSpinBox* overlapSpin;
    kfpcsInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"KFPCS参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        neighborNum = new QSpinBox(this);
        neighborNum->setRange(5, 100);
        neighborNum->setValue(25);
        neighborNum->setSingleStep(1);
        overlapSpin = new QDoubleSpinBox(this);
        overlapSpin->setRange(0.0, 1.0);
        overlapSpin->setValue(0.5);
        overlapSpin->setSingleStep(0.1);
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);
        maxIterationsSpin->setValue(100);
        formLayout->addRow(u8"最大迭代次数:", maxIterationsSpin);
        formLayout->addRow(u8"验证采样数量:", neighborNum);
        formLayout->addRow(u8"近似重叠度:", overlapSpin);
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
        formLayout->addRow(buttonBox);

        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    int getNeighborNum() const { return neighborNum->value(); }
    double getOverlap() const { return overlapSpin->value(); }
};

//FPFH参数设置：K近邻K值；体素大小；最大迭代次数；最大对应点距离；对应点关系
class fpfhInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* voxelSpin;
    QDoubleSpinBox* maxDistanceSpin;
    QCheckBox* reciprocalCorrespondences;
    QSpinBox* kSpin;
    fpfhInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"FPFH参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);

        // 最大迭代次数（整数输入）
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);
        maxIterationsSpin->setValue(100);

        voxelSpin = new QDoubleSpinBox(this);
        voxelSpin->setRange(0.1, 100.0);
        voxelSpin->setValue(1.0);
        voxelSpin->setSingleStep(0.1);

        reciprocalCorrespondences = new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        kSpin = new QSpinBox(this);
        kSpin->setRange(1, 100);
        kSpin->setValue(10);
        kSpin->setSingleStep(1);

        maxDistanceSpin = new QDoubleSpinBox(this);
        maxDistanceSpin->setRange(0.0, 100.0);
        maxDistanceSpin->setValue(0.05);
        maxDistanceSpin->setSingleStep(0.01);
        formLayout->addRow(u8"最大迭代次数:", maxIterationsSpin);
        formLayout->addRow(u8"体素大小:", voxelSpin);
        formLayout->addRow(u8"最大对应点距离:", maxDistanceSpin);
        formLayout->addRow(u8"对应点关系:", reciprocalCorrespondences);
        formLayout->addRow(u8"K近邻K值:", kSpin);

        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
        formLayout->addRow(buttonBox);

        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getVoxel() const { return voxelSpin->value(); }
    double getMaxDistance() const { return maxDistanceSpin->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
    int getK() const { return kSpin->value(); }
};

//随机采样参数设置：采样点数
class randomInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* numSpin;
    randomInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"随机采样参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        numSpin = new QSpinBox(this);
        numSpin->setRange(1, 1000000);
        numSpin->setValue(20000);
        formLayout->addRow(u8"采样点数:", numSpin);
        QDialogButtonBox* buttonBox = new QDialogButtonBox();
        buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
    }
    int getNum() const { return numSpin->value(); }
};

//ISS降采样参数设置：关键区域半径；显著半径；ISS半径；关键区域采样率；非关键区域采样率
class issInputDialog : public QDialog {
    Q_OBJECT
public:
    QDoubleSpinBox* ISSRadiusSpin;
    QDoubleSpinBox* keyRadiusSpin;
    QDoubleSpinBox* salientRadiusSpin;
    QDoubleSpinBox* keySampleSpin;
    QDoubleSpinBox* nonKeySampleSpin;
    issInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"ISS降采样参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        ISSRadiusSpin = new QDoubleSpinBox(this);
        ISSRadiusSpin->setRange(0.0, 100.0);
        ISSRadiusSpin->setValue(1.0);
        keyRadiusSpin = new QDoubleSpinBox(this);
        keyRadiusSpin->setRange(0.0, 100.0);
        keyRadiusSpin->setValue(1.0);
        salientRadiusSpin = new QDoubleSpinBox(this);
        salientRadiusSpin->setRange(0.0, 100.0);
        salientRadiusSpin->setValue(1.0);
        keySampleSpin = new QDoubleSpinBox(this);
        keySampleSpin->setRange(0.0, 1.0);
        keySampleSpin->setValue(0.5);
        nonKeySampleSpin = new QDoubleSpinBox(this);
        nonKeySampleSpin->setRange(0.0, 1.0);
        nonKeySampleSpin->setValue(0.5);
        formLayout->addRow(u8"ISS半径:", ISSRadiusSpin);
        formLayout->addRow(u8"显著半径:", keyRadiusSpin);
        formLayout->addRow(u8"关键区域半径:", salientRadiusSpin);
        formLayout->addRow(u8"关键区域采样率:", keySampleSpin);
        formLayout->addRow(u8"非关键区域采样率:", nonKeySampleSpin);
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
        formLayout->addRow(buttonBox);
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }
    double getISSRadius() const { return ISSRadiusSpin->value(); }
    double getKeyRadius() const { return keyRadiusSpin->value(); }
    double getSalientRadius() const { return salientRadiusSpin->value(); }
    double getKeySample() const { return keySampleSpin->value(); }
    double getNonKeySample() const { return nonKeySampleSpin->value(); }
};

//体素降采样参数设置：体素大小
class voxelInputDialog : public QDialog {
    Q_OBJECT
public:
    QDoubleSpinBox* voxelSpin;
    voxelInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"体素降采样参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        voxelSpin = new QDoubleSpinBox(this);
        voxelSpin->setRange(0.0, 100.0);
        voxelSpin->setValue(1.0);
        formLayout->addRow(u8"体素大小:", voxelSpin);
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
        formLayout->addRow(buttonBox);
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }
    double getVoxel() const { return voxelSpin->value(); }
};

//曲率降采样参数设置：采样点数
class curvatureInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* sampleNum;
    curvatureInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"曲率降采样参数设置");
        // 创建布局和控件
        QFormLayout* formLayout = new QFormLayout(this);
        sampleNum = new QSpinBox(this);
        sampleNum->setRange(0.0, 100.0);
        sampleNum->setValue(1.0);
        formLayout->addRow(u8"采样率:", sampleNum);
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
        formLayout->addRow(buttonBox);
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }
    int getSampleRate() const { return sampleNum->value(); }
};


//// 使用示例
//int main(int argc, char* argv[]) {
//    QApplication app(argc, argv);
//
//    InputDialog dialog;
//    if (dialog.exec() == QDialog::Accepted) {
//        qDebug() << "最大迭代次数:" << dialog.getMaxIterations();
//        qDebug() << "尺度大小:" << dialog.getScale();
//        qDebug() << "最大对应点距离:" << dialog.getMaxDistance();
//    }
//    return 0;
//}