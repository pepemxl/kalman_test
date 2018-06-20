#include "mainwindow.h"
#include "ui_mainwindow.h"

#define drawCross(center,color,d) cv::line(img,cv::Point(center.x-d,center.y-d),cv::Point(center.x+d,center.y+d),color,1,cv::LINE_AA,0);cv::line(img,cv::Point(center.x+d,center.y-d),cv::Point(center.x-d,center.y+d),color,1,cv::LINE_AA,0);

static inline cv::Point calcPoint(cv::Point2f center, double R, double angle){
    return center + cv::Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}
static void help(){
    printf( " Velocidad constante.\n Measurement es el ángulo más ruido gaussiano.\n En azul la conexión entre el punto real y el estimado,\n En verde la conexión entre el punto real y el medido.\n");
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setLSide(500);
    ui->doubleSpinBoxRadio->setMaximum((double)this->getLSide()/2.0);
    ui->doubleSpinBoxRadio->setValue((double)this->getLSide()/3.0);
    this->setFlagStopTest01(true);
    this->setFlagStopTest02(true);
    this->setTimeSleep(ui->spinBoxTimeSleep->value());
    this->setDt(ui->doubleSpinBoxDt->value());
    this->setMedia(ui->doubleSpinBoxMedia->value());
    this->setDesviacionEstandar(ui->doubleSpinBoxDesviacionEstandar->value());
    ui->labelMedia->setText("<html>&mu; de X<sub>0</sub></html>");
    ui->labelDesviacionEstandar->setText("<html>&sigma; de X<sub>0</sub></html>");
}

MainWindow::~MainWindow()
{
    this->setFlagStopTest01(true);
    this->setFlagStopTest02(true);
    close();
    qApp->quit();
    delete ui;
}

bool MainWindow::getFlagStopTest02() const
{
    return flagStopTest02;
}

void MainWindow::setFlagStopTest02(bool value)
{
    flagStopTest02 = value;
}

int MainWindow::getCurrentNumberOfTest() const
{
    return currentNumberOfTest;
}

void MainWindow::setCurrentNumberOfTest(int value)
{
    currentNumberOfTest = value;
}

bool MainWindow::getFlagStopAll() const
{
    return flagStopAll;
}

void MainWindow::setFlagStopAll(bool value)
{
    flagStopAll = value;
}

float MainWindow::getRadio() const
{
    return radio;
}

void MainWindow::setRadio(float value)
{
    radio = value;
}

int MainWindow::getLSide() const
{
    return lSide;
}

void MainWindow::setLSide(int value)
{
    lSide = value;
}

float MainWindow::getDt() const
{
    return dt;
}

void MainWindow::setDt(float value)
{
    dt = value;
}

float MainWindow::getDesviacionEstandar() const
{
    return desviacionEstandar;
}

void MainWindow::setDesviacionEstandar(float value)
{
    desviacionEstandar = value;
    this->setVarianza(value*value);
}

float MainWindow::getVarianza() const
{
    return varianza;
}

void MainWindow::setVarianza(float value)
{
    varianza = value;
}

float MainWindow::getMedia() const
{
    return media;
}

void MainWindow::setMedia(float value)
{
    media = value;
}

unsigned int MainWindow::getTimeSleep() const
{
    return timeSleep;
}

void MainWindow::setTimeSleep(unsigned int value)
{
    timeSleep = value;
}

char MainWindow::getCode() const
{
    return code;
}

void MainWindow::setCode(char value)
{
    code = value;
}

bool MainWindow::getFlagStopTest01() const
{
    return flagStopTest01;
}

void MainWindow::setFlagStopTest01(bool value)
{
    flagStopTest01 = value;
}

void MainWindow::udpateLabelTest01(cv::Mat &img){
    cv::Mat src;
    float tempScale = .55;
    src = img.clone();
    if(src.size().width > 0){
        tempScale = (float)ui->labelTest01->geometry().width()/(float)src.size().width;
        cv::resize(src,src,cv::Size(), tempScale, tempScale,cv::INTER_CUBIC);
        cvtColor(src,src,CV_BGR2RGB);
        QImage dest((const uchar *) src.data, src.cols, src.rows, src.step, QImage::Format_RGB888);
        dest.bits();
        ui->labelTest01->setPixmap(QPixmap::fromImage(dest));
        QApplication::processEvents();
    }
}
void MainWindow::udpateLabelTest02(cv::Mat &img){
    cv::Mat src;
    float tempScale = .55;
    src = img.clone();
    if(src.size().width > 0){
        tempScale = (float)ui->labelTest01->geometry().width()/(float)src.size().width;
        cv::resize(src,src,cv::Size(), tempScale, tempScale,cv::INTER_CUBIC);
        cvtColor(src,src,CV_BGR2RGB);
        QImage dest((const uchar *) src.data, src.cols, src.rows, src.step, QImage::Format_RGB888);
        dest.bits();
        ui->labelTest01->setPixmap(QPixmap::fromImage(dest));
        QApplication::processEvents();
    }
}

void MainWindow::test_kalman_01(){
    std::cout << std::fixed;
    this->setFlagStopTest01(false);
    cv::Mat img(500, 500, CV_8UC3);
    cv::KalmanFilter KF(2, 1, 0); //!< Sistema
    cv::Mat state(2, 1, CV_32F); //!< Variable que guarda el estado del sistema,i.e., (phi, omega), posición y velocidad
    cv::Mat processNoise(2, 1, CV_32F);//
    cv::Mat measurement = cv::Mat::zeros(1, 1, CV_32F); //!< Variable que guarda las mediciones realizadas
    this->setCode((char)-1);
    if(!this->getFlagStopTest01()){
        //!< Inicializamos el arreglo de estados aleatoriamente, con una media y una desviación estandar
        //cv::randn(state,cv::Scalar::all(0),cv::Scalar::all(0.1));
        cv::randn(state,cv::Scalar::all(this->getMedia()),cv::Scalar::all(this->getDesviacionEstandar()));
        //!< Se define la matriz de transición
        KF.transitionMatrix = (cv::Mat_<float>(2, 2) << 1, this->getDt(), 0, 1);
        //!< Crea matriz Identidad
        cv::setIdentity(KF.measurementMatrix);
        //!< Crea matriz Identidad para la covarianza
        cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
        //!< Crea matriz Identidad con covarianza
        cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
        //!< Crea matriz Identidad para el error
        cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
        //!< Inicializa vector corregido con valor aleatorio
        cv::randn(KF.statePost,cv::Scalar::all(0),cv::Scalar::all(0.1));
        //cv::randn(KF.statePost, cv::Scalar::all(this->getMedia()), cv::Scalar::all(this->getDesviacionEstandar()));
        cv::Point2f center(img.cols*0.5f, img.rows*0.5f);
        while(!this->getFlagStopTest01()){
            float R = this->getRadio();
            double stateAngle = state.at<float>(0);
            cv::Point statePt = calcPoint(center, R, stateAngle);

            cv::Mat prediction = KF.predict();
            double predictAngle = prediction.at<float>(0);
            std::cout << "predictAngle: " << std::setprecision(6) << predictAngle << std::endl;
            cv::Point predictPt = calcPoint(center, R, predictAngle);

            cv::randn( measurement, cv::Scalar::all(0), cv::Scalar::all(KF.measurementNoiseCov.at<float>(0)));

            // generate measurement
            measurement += KF.measurementMatrix*state;

            double measAngle = measurement.at<float>(0);
            cv::Point measPt = calcPoint(center, R, measAngle);

            // plot points
            img = cv::Scalar::all(255);
            drawCross( statePt, cv::Scalar(255,255,255), 3 );
            drawCross( measPt, cv::Scalar(0,0,255), 3 );
            drawCross( predictPt, cv::Scalar(255,0,0), 3 );
            cv::line( img, statePt, measPt, cv::Scalar(0,255,0), 3, cv::LINE_AA, 0 );
            cv::line( img, statePt, predictPt, cv::Scalar(255,0,0), 3, cv::LINE_AA, 0 );

            if(cv::theRNG().uniform(0,4) != 0)
                KF.correct(measurement);

            cv::randn( processNoise, cv::Scalar(0),cv:: Scalar::all(std::sqrt(KF.processNoiseCov.at<float>(0, 0))));
            state = KF.transitionMatrix*state + processNoise;
            this->udpateLabelTest01(img);
            std::this_thread::sleep_for(std::chrono::milliseconds(this->getTimeSleep()));
        }
    }
    return;
}

/**
 *  \brief El filtro de Kalman intenta estimar el estado $\mathbb{x}\in \mathbb{R}^{n}$ del
 * sistema que es governado por la ecuación:
 *                        $$x_{k}=Ax_{k-1}+Bu_{k}+w_{k-1}$$
 * con medida
 *                        $$z_{k}= Hx_{k}+v_{k}$$
 * Donde $w_{k-1},v_{k}$ representan recpectivamente el ruido del proceso y de la medición.
 * Se asume que tienen media 0, ruido blanco con matrices de covarianza Q y R.
 * La matriz $A$ es llamada matriz de transición, y relaciona el estado actual con el estado previo,
 * es decir, $x_{k} con x_{k-1}$, si no hay ruido presente.
 * La matriz $A\in Math(n\times n)$.
 * B es una matriz opcional que relaciona la entrada de control $u_{k}\in\mathbb{R}^{l}$ del estado
 * $x_{k}$, en el caso de trackeo que usaremos por el momento no existe dicha variable de control,
 * por lo cual todo el termino $Bu_{k}$ es retirado de la ecuación.
 * La matriz $H\in Math(n\times l)$ relaciona la medición $z_{k}$ con el estado $x_{k}$.
 *
 * El filtro de kalman mantiene dos estimadores todo el tiempo:
 * \begin{itemize}
 *  \item $\hat{x}(k|k-1)$, estima el estado al tiempo $k$, dado que se conoce el proceso al tiempo
 * $k-1$, es decir, es una estimación a priori del estado al tiempo $k$.
 *  \item $\hat{x}(k|k)$, estima el proceso al tiempo $k$ dada la medición $z_{k}$, es decir,
 * es una estimación a posteiori del estado al tiempo $k$.
 * \end{itemize}
 *
 * Además mantiene dos matrices de covarianza del error del estado estimado:
 * \begin{itemize}
 *  \item $P(k|k-1)$, que es una estimación a priori de la covarianza del error de $\hat{x}(k|k-1)$.
 *  \item $P(k|k)$, que es una estimación a posteriori de la covarianza del error de $\hat{x}(k|k)$.
 * \end{itemize}
 *
 * Kalman es un estimador cuadratico medio recursivo de minimos, que opera en dos fases sobre
 * cada paso de tiempo $k$.
 * En la primer fase se predice el siguiente estado estimado $\hat{x}(k|k-1)$ usando el previo.
 * En la segunda fase se encuentra la corrección del estimado usando la medición, para entonces
 * obtener $\hat{x}(k|k)$.
 *
 * En un inicio se considera que $\hat{x}(1|1)$ y $P(1|1)$ son conocidos.
 *
 * Para mantener dichas estimaciones, se realizan las siguientes operaciones:
 * 1.- Predicción del estado:
 *                  $$\hat{x}(k|k-1) = A\cdot\hat{x}(k-1|k-1)$$
 * 2.- Predicción de la covarianza del error:
 *                  $$P(k|k-1) = A\cdot P(k-1|k-1)\cdot A^{T}+Q$$
 *
 * En el paso de correción:
 * 3.- Se hace medición de la predicción:
 *            $$\hat{z}(k|k-1) = H\cdot \hat{x}(k|k-1)$$
 * 4.- Se calcula el residuo:
 *            $$r_{k} = z_{k} - \hat{z}(k|k-1)$$
 * 5.- Se hace la medición de la covarianza predicha:
 *            $$S_{k} = H\cdot P(k|k-1)\cdot H^{T}+R$$
 * 6.- Se calcula la ganancia de Kalman:
 *            $$W_{k}=P(k|k-1)\cdot H^{T}\cdot S^{-1}$$
 * 7.- Se actualiza el estado:
 *            $$\hat{x}(k|k) = \hat{x}(k|k-1)+W_{k}r_{k}$$
 * 8.- Se actualiza la covarianza del error:
 *            $$P(k|k) = P(k|k-1)-W_{k}\cdot S_{k}\cdot W_{k}^{T}$$
 *
 * Ahora para inicializar el filtro de Kalman es necesario definir la matriz de transición $A$,
 * la matriz de medición de estados $H$, las dos matrices de covarianza $R$ y $Q$, y en cada paso
 * de tiempo alimentar al filtro con una medición $z_{k}$.
 * La ventaja de este tipo de sistemas es que es facilmente intercambiable por otro estimador,
 * como podria ser un filtro de particulas o un filtro extendido de Kalman.
 *
 * Ahora para el problema de tracking definiremos POM(x,y) un Probabilistic Occupancy Map de la
 * escena con un grid de dimensiones $W\times H$ donde $0<=x<W$ y $0<=y<H$ y $0 <= POM(x,y) <=1$.
 * Ahora definimos la siguiente función indicatriz G
 *                 $$G(x,y) = \left{\begin{itemize}
 * \item 1, si POM(x,y) >= \theta
 * \item 0, en cualquier otro caso\end{itemize}\right.$$
 * Cada posición del gris donde $G(x,y) = 1$ es considerada una medición $m$ y $M_{k}$ es el
 * conjunto de todas las mediciones posibles en un frame dado $k$.
 * $dt = \frac{1}{fps}$
 * $$
*/

void MainWindow::test_kalman_02(){
    std::cout << std::fixed;
    this->setFlagStopTest02(false);
    cv::Mat img(500, 500, CV_8UC3);
    //cv::KalmanFilter KF(2, 1, 0); //!< Sistema
    cv::KalmanFilter KF(4, 2, 0); //!< Sistema ($\hat{x}_{k},\hat{z}_{k}$
    cv::Mat state(4, 1, CV_32F); //!< Variable que guarda el estado del sistema,i.e., (\phi_{x},\phi_{y}, \omega_{x},\omega_{y}), posición y velocidad
    cv::Mat processNoise(4, 1, CV_32F);//
    cv::Mat measurement = cv::Mat::zeros(2, 1, CV_32F); //!< Variable que guarda las mediciones realizadas
    this->setCode((char)-1);
    if(!this->getFlagStopTest02()){
        //!< Inicializamos el arreglo de estados aleatoriamente, con una media y una desviación estandar
        //cv::randn(state,cv::Scalar::all(0),cv::Scalar::all(0.1));
        cv::randn(state,cv::Scalar::all(this->getMedia()),cv::Scalar::all(this->getDesviacionEstandar()));
        //!< Se define la matriz de transición
        KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0,this->getDt(), 0, 0,1,0,this->getDt(),0,0,1,0,0,0,0,1);
        std::cout << KF.transitionMatrix << std::endl;
        //!< Crea matriz Identidad
        //cv::setIdentity(KF.measurementMatrix);
        KF.measurementMatrix = (cv::Mat_<float>(2, 4) << 1,0,0,0,0,1,0,0);
        //std::cout << KF.transitionMatrix << std::endl;
        //!< Crea matriz Identidad para la covarianza
        cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
        //!< Crea matriz Identidad con covarianza
        cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
        //!< Crea matriz Identidad para el error
        cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
        //!< Inicializa vector corregido con valor aleatorio
        cv::randn(KF.statePost,cv::Scalar::all(0),cv::Scalar::all(0.1));
        //cv::randn(KF.statePost, cv::Scalar::all(this->getMedia()), cv::Scalar::all(this->getDesviacionEstandar()));
        cv::Point2f center(img.cols*0.5f, img.rows*0.5f);
        while(!this->getFlagStopTest02()){
            float R = this->getRadio();
            //double stateAngle = state.at<float>(0);
            double stateAngle = std::atan2(state.at<float>(1),state.at<float>(0));
            std::cout << "stateAngle: " << std::setprecision(6) << stateAngle;// << std::endl;
            cv::Point statePt = calcPoint(center, R, stateAngle);

            cv::Mat prediction = KF.predict();
            //double predictAngle = prediction.at<float>(0);
            double predictAngle = std::atan2(prediction.at<float>(1),prediction.at<float>(0));
            std::cout << " predictAngle: " << std::setprecision(6) << predictAngle;// << std::endl;
            cv::Point predictPt = calcPoint(center, R, predictAngle);

            cv::randn( measurement, cv::Scalar::all(0), cv::Scalar::all(KF.measurementNoiseCov.at<float>(0)));

            // generate measurement
            measurement += KF.measurementMatrix*state;

            //double measAngle = measurement.at<float>(0);
            double measAngle = std::atan2(measurement.at<float>(1),measurement.at<float>(0));
            std::cout << " measAngle: " << std::setprecision(6) << measAngle << std::endl;
            cv::Point measPt = calcPoint(center, R, measAngle);

            // plot points
            img = cv::Scalar::all(255);
            drawCross( statePt, cv::Scalar(255,255,255), 3 );
            drawCross( measPt, cv::Scalar(0,0,255), 3 );
            drawCross( predictPt, cv::Scalar(255,0,0), 3 );
            cv::line( img, statePt, measPt, cv::Scalar(0,255,0), 3, cv::LINE_AA, 0 );
            cv::line( img, statePt, predictPt, cv::Scalar(255,0,0), 3, cv::LINE_AA, 0 );

            if(cv::theRNG().uniform(0,4) != 0)
                KF.correct(measurement);

            cv::randn( processNoise, cv::Scalar(0),cv:: Scalar::all(std::sqrt(KF.processNoiseCov.at<float>(0, 0))));
            state = KF.transitionMatrix*state + processNoise;
            this->udpateLabelTest02(img);
            std::this_thread::sleep_for(std::chrono::milliseconds(this->getTimeSleep()));
        }
    }
    return;
}

void MainWindow::on_pushButtonTest01_clicked()
{
    if(this->getFlagStopTest01()){
        this->test_kalman_01();
    }else{
        this->setFlagStopTest01(true);
    }
}

void MainWindow::on_pushButtonTest02_clicked()
{
    if(this->getFlagStopTest02()){
        this->test_kalman_02();
    }else{
        this->setFlagStopTest02(true);
    }
}

void MainWindow::on_spinBoxTimeSleep_valueChanged(int arg1)
{
    this->setTimeSleep(arg1);
}

void MainWindow::on_doubleSpinBoxMedia_valueChanged(double arg1)
{
    this->setMedia(arg1);
}

void MainWindow::on_doubleSpinBoxDesviacionEstandar_valueChanged(double arg1)
{
    this->setDesviacionEstandar(arg1);
}

void MainWindow::on_doubleSpinBoxDt_valueChanged(double arg1)
{
    this->setDt(arg1);
}

void MainWindow::on_doubleSpinBoxRadio_valueChanged(double arg1)
{
    this->setRadio(arg1);
}


