#include <QApplication>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>  
#include <QDialogButtonBox>
#include <QPushButton>

//�Ľ�gicp�����
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
        setWindowTitle(u8"�Ľ�GICP��������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences = new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // �������������������룩
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // ����1-1000
        maxIterationsSpin->setValue(100);        // Ĭ��ֵ

        // ��һ�߶ȴ�С�����������룩
        scaleSpin1 = new QDoubleSpinBox(this);
        scaleSpin1->setRange(0.1, 10.0);         // ����0.1-10.0
        scaleSpin1->setValue(1.0);               // Ĭ��ֵ
        scaleSpin1->setSingleStep(0.1);          // ����0.1

        // �ڶ��߶ȴ�С�����������룩
        scaleSpin2 = new QDoubleSpinBox(this);
        scaleSpin2->setRange(0.1, 10.0);         // ����0.1-10.0
        scaleSpin2->setValue(1.0);               // Ĭ��ֵ
        scaleSpin2->setSingleStep(0.1);          // ����0.1

        // �����߶ȴ�С�����������룩
        scaleSpin3 = new QDoubleSpinBox(this);
        scaleSpin3->setRange(0.1, 10.0);         // ����0.1-10.0
        scaleSpin3->setValue(1.0);               // Ĭ��ֵ
        scaleSpin3->setSingleStep(0.1);          // ����0.1

        // ����Ӧ����루���������룩
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // ����0-100
        distanceSpin->setValue(25.0);           // Ĭ��ֵ
        distanceSpin->setDecimals(2);           // ��λС��

        // ��ӵ�������
        formLayout->addRow(u8"����������:", maxIterationsSpin);
        formLayout->addRow(u8"��һ�߶ȴ�С:", scaleSpin1);
        formLayout->addRow(u8"�ڶ��߶ȴ�С:", scaleSpin2);
        formLayout->addRow(u8"�����߶ȴ�С:", scaleSpin3);
        formLayout->addRow(u8"����Ӧ�����:", distanceSpin);
        formLayout->addRow(u8"���໥��ϵ��", reciprocalCorrespondences);
        

        // ��Ӱ�ť
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // �����źŲ�
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // ��ȡ����ֵ�Ĺ��з���
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getScale1() const { return scaleSpin1->value(); }
    double getScale2() const { return scaleSpin2->value(); }
    double getScale3() const { return scaleSpin3->value(); }
    double getMaxDistance() const { return distanceSpin->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
};


//gicp�����
class gicpInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* scaleSpin;
    QDoubleSpinBox* distanceSpin;
    QCheckBox* reciprocalCorrespondences;
    gicpInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"GICP��������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences= new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // �������������������룩
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // ����1-1000
        maxIterationsSpin->setValue(100);        // Ĭ��ֵ

        // �߶ȴ�С�����������룩
        scaleSpin = new QDoubleSpinBox(this);
        scaleSpin->setRange(0.1, 10.0);         // ����0.1-10.0
        scaleSpin->setValue(1.0);               // Ĭ��ֵ
        scaleSpin->setSingleStep(0.1);          // ����0.1

        // ����Ӧ����루���������룩
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // ����0-100
        distanceSpin->setValue(25.0);           // Ĭ��ֵ
        distanceSpin->setDecimals(2);           // ��λС��

        // ��ӵ�������
        formLayout->addRow(u8"����������:", maxIterationsSpin);
        formLayout->addRow(u8"�߶ȴ�С:", scaleSpin);
        formLayout->addRow(u8"����Ӧ�����:", distanceSpin);
        formLayout->addRow(u8"���໥��ϵ��", reciprocalCorrespondences);

        // ��Ӱ�ť
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // �����źŲ�
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // ��ȡ����ֵ�Ĺ��з���
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getScale() const { return scaleSpin->value(); }
    double getMaxDistance() const { return distanceSpin->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
};

//icp�����
class icpInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* scaleSpin;
    QDoubleSpinBox* distanceSpin;
    QCheckBox* reciprocalCorrespondences;
    icpInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"ICP��������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences = new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // �������������������룩
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // ����1-1000
        maxIterationsSpin->setValue(100);        // Ĭ��ֵ

        // �߶ȴ�С�����������룩
        scaleSpin = new QDoubleSpinBox(this);
        scaleSpin->setRange(0.1, 10.0);         // ����0.1-10.0
        scaleSpin->setValue(1.0);               // Ĭ��ֵ
        scaleSpin->setSingleStep(0.1);          // ����0.1

        // ����Ӧ����루���������룩
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // ����0-100
        distanceSpin->setValue(25.0);           // Ĭ��ֵ
        distanceSpin->setDecimals(2);           // ��λС��

        // ��ӵ�������
        formLayout->addRow(u8"����������:", maxIterationsSpin);
        formLayout->addRow(u8"�߶ȴ�С:", scaleSpin);
        formLayout->addRow(u8"����Ӧ�����:", distanceSpin);
        formLayout->addRow(u8"���໥��ϵ��", reciprocalCorrespondences);

        // ��Ӱ�ť
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // �����źŲ�
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // ��ȡ����ֵ�Ĺ��з���
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
        setWindowTitle(u8"NICP��������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences = new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // �������������������룩
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // ����1-1000
        maxIterationsSpin->setValue(100);        // Ĭ��ֵ

        // �߶ȴ�С�����������룩
        scaleSpin = new QDoubleSpinBox(this);
        scaleSpin->setRange(0.1, 10.0);         // ����0.1-10.0
        scaleSpin->setValue(1.0);               // Ĭ��ֵ
        scaleSpin->setSingleStep(0.1);          // ����0.1

        // ����Ӧ����루���������룩
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // ����0-100
        distanceSpin->setValue(25.0);           // Ĭ��ֵ
        distanceSpin->setDecimals(2);           // ��λС��

        //�������ھ���
        neighborNum = new QSpinBox(this);
        neighborNum->setRange(5, 100);
        neighborNum->setValue(25);
        neighborNum->setSingleStep(1);

        // ��ӵ�������
        formLayout->addRow(u8"����������:", maxIterationsSpin);
        formLayout->addRow(u8"�߶ȴ�С:", scaleSpin);
        formLayout->addRow(u8"����Ӧ�����:", distanceSpin);
        formLayout->addRow(u8"�������ھ���:", neighborNum);
        formLayout->addRow(u8"���໥��ϵ��", reciprocalCorrespondences);

        // ��Ӱ�ť
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // �����źŲ�
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // ��ȡ����ֵ�Ĺ��з���
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getScale() const { return scaleSpin->value(); }
    double getMaxDistance() const { return distanceSpin->value(); }
    int getNeighborNum() const { return neighborNum->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
};
//nonlinearIcp�����
class nonlinearIcpInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* scaleSpin;
    QDoubleSpinBox* distanceSpin;
    QCheckBox* reciprocalCorrespondences;
    QSpinBox* neighborNum;
    nonlinearIcpInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"non-linear ICP��������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);
        reciprocalCorrespondences = new QCheckBox(this);
        reciprocalCorrespondences->setChecked(true);

        // �������������������룩
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // ����1-1000
        maxIterationsSpin->setValue(100);        // Ĭ��ֵ

        // �߶ȴ�С�����������룩
        scaleSpin = new QDoubleSpinBox(this);
        scaleSpin->setRange(0.1, 10.0);         // ����0.1-10.0
        scaleSpin->setValue(1.0);               // Ĭ��ֵ
        scaleSpin->setSingleStep(0.1);          // ����0.1

        // ����Ӧ����루���������룩
        distanceSpin = new QDoubleSpinBox(this);
        distanceSpin->setRange(0.0, 100.0);     // ����0-100
        distanceSpin->setValue(25.0);           // Ĭ��ֵ
        distanceSpin->setDecimals(2);           // ��λС��

        //�������ھ���
        neighborNum = new QSpinBox(this);
        neighborNum->setRange(5, 100);
        neighborNum->setValue(25);
        neighborNum->setSingleStep(1);

        // ��ӵ�������
        formLayout->addRow(u8"����������:", maxIterationsSpin);
        formLayout->addRow(u8"�߶ȴ�С:", scaleSpin);
        formLayout->addRow(u8"����Ӧ�����:", distanceSpin);
        formLayout->addRow(u8"�������ھ���:", neighborNum);
        formLayout->addRow(u8"���໥��ϵ��", reciprocalCorrespondences);

        // ��Ӱ�ť
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // �����źŲ�
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // ��ȡ����ֵ�Ĺ��з���
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getScale() const { return scaleSpin->value(); }
    double getMaxDistance() const { return distanceSpin->value(); }
    int getNeighborNum() const { return neighborNum->value(); }
    bool getReciprocalCorrespondences() const { return reciprocalCorrespondences->isChecked(); }
};

//NDT�����:����������������ֱ���
class NDTInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* voxelSpin;
    NDTInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"NDT��������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);

        // �������������������룩
        maxIterationsSpin = new QSpinBox(this);
        maxIterationsSpin->setRange(1, 1000);    // ����1-1000
        maxIterationsSpin->setValue(100);        // Ĭ��ֵ

        // �߶ȴ�С�����������룩
        voxelSpin = new QDoubleSpinBox(this);
        voxelSpin->setRange(0.1, 100.0);         // ����0.1-10.0
        voxelSpin->setValue(1.0);               // Ĭ��ֵ
        voxelSpin->setSingleStep(0.1);          // ����0.1

        

        // ��ӵ�������
        formLayout->addRow(u8"����������:", maxIterationsSpin);
        formLayout->addRow(u8"����ֱ���:", voxelSpin);

        // ��Ӱ�ť
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);

        formLayout->addRow(buttonBox);

        // �����źŲ�
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    // ��ȡ����ֵ�Ĺ��з���
    int getMaxIterations() const { return maxIterationsSpin->value(); }
    double getVoxel() const { return voxelSpin->value(); }
};

//fpcs�������ã���������������֤���������������ص���
class fpcsInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QSpinBox* neighborNum;
    QDoubleSpinBox* overlapSpin;
    fpcsInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"FPCS��������");
        // �������ֺͿؼ�
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
        formLayout->addRow(u8"����������:", maxIterationsSpin);
        formLayout->addRow(u8"��֤��������:", neighborNum);
        formLayout->addRow(u8"�����ص���:", overlapSpin);
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

//k4pcs�������ã���������������֤���������������ص���
class kfpcsInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QSpinBox* neighborNum;
    QDoubleSpinBox* overlapSpin;
    kfpcsInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"KFPCS��������");
        // �������ֺͿؼ�
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
        formLayout->addRow(u8"����������:", maxIterationsSpin);
        formLayout->addRow(u8"��֤��������:", neighborNum);
        formLayout->addRow(u8"�����ص���:", overlapSpin);
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

//FPFH�������ã�K����Kֵ�����ش�С������������������Ӧ����룻��Ӧ���ϵ
class fpfhInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* maxIterationsSpin;
    QDoubleSpinBox* voxelSpin;
    QDoubleSpinBox* maxDistanceSpin;
    QCheckBox* reciprocalCorrespondences;
    QSpinBox* kSpin;
    fpfhInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"FPFH��������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);

        // �������������������룩
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
        formLayout->addRow(u8"����������:", maxIterationsSpin);
        formLayout->addRow(u8"���ش�С:", voxelSpin);
        formLayout->addRow(u8"����Ӧ�����:", maxDistanceSpin);
        formLayout->addRow(u8"��Ӧ���ϵ:", reciprocalCorrespondences);
        formLayout->addRow(u8"K����Kֵ:", kSpin);

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

//��������������ã���������
class randomInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* numSpin;
    randomInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"���������������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);
        numSpin = new QSpinBox(this);
        numSpin->setRange(1, 1000000);
        numSpin->setValue(20000);
        formLayout->addRow(u8"��������:", numSpin);
        QDialogButtonBox* buttonBox = new QDialogButtonBox();
        buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
    }
    int getNum() const { return numSpin->value(); }
};

//ISS�������������ã��ؼ�����뾶�������뾶��ISS�뾶���ؼ���������ʣ��ǹؼ����������
class issInputDialog : public QDialog {
    Q_OBJECT
public:
    QDoubleSpinBox* ISSRadiusSpin;
    QDoubleSpinBox* keyRadiusSpin;
    QDoubleSpinBox* salientRadiusSpin;
    QDoubleSpinBox* keySampleSpin;
    QDoubleSpinBox* nonKeySampleSpin;
    issInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"ISS��������������");
        // �������ֺͿؼ�
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
        formLayout->addRow(u8"ISS�뾶:", ISSRadiusSpin);
        formLayout->addRow(u8"�����뾶:", keyRadiusSpin);
        formLayout->addRow(u8"�ؼ�����뾶:", salientRadiusSpin);
        formLayout->addRow(u8"�ؼ����������:", keySampleSpin);
        formLayout->addRow(u8"�ǹؼ����������:", nonKeySampleSpin);
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

//���ؽ������������ã����ش�С
class voxelInputDialog : public QDialog {
    Q_OBJECT
public:
    QDoubleSpinBox* voxelSpin;
    voxelInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"���ؽ�������������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);
        voxelSpin = new QDoubleSpinBox(this);
        voxelSpin->setRange(0.0, 100.0);
        voxelSpin->setValue(1.0);
        formLayout->addRow(u8"���ش�С:", voxelSpin);
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
        formLayout->addRow(buttonBox);
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }
    double getVoxel() const { return voxelSpin->value(); }
};

//���ʽ������������ã���������
class curvatureInputDialog : public QDialog {
    Q_OBJECT
public:
    QSpinBox* sampleNum;
    curvatureInputDialog(QWidget* parent = nullptr) : QDialog(parent) {
        setWindowTitle(u8"���ʽ�������������");
        // �������ֺͿؼ�
        QFormLayout* formLayout = new QFormLayout(this);
        sampleNum = new QSpinBox(this);
        sampleNum->setRange(0.0, 100.0);
        sampleNum->setValue(1.0);
        formLayout->addRow(u8"������:", sampleNum);
        QDialogButtonBox* buttonBox = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
            Qt::Horizontal, this);
        formLayout->addRow(buttonBox);
        connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }
    int getSampleRate() const { return sampleNum->value(); }
};


//// ʹ��ʾ��
//int main(int argc, char* argv[]) {
//    QApplication app(argc, argv);
//
//    InputDialog dialog;
//    if (dialog.exec() == QDialog::Accepted) {
//        qDebug() << "����������:" << dialog.getMaxIterations();
//        qDebug() << "�߶ȴ�С:" << dialog.getScale();
//        qDebug() << "����Ӧ�����:" << dialog.getMaxDistance();
//    }
//    return 0;
//}