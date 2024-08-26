using System;
using System.IO.Ports;
using System.Linq;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using System.Diagnostics;

namespace PID통신_model
{
    public partial class Form1 : Form
    {
        SerialPort serialPort;
        string dataIn;
        private Stopwatch stopwatch = new Stopwatch();
        private double timeInterval = 0.005;  // 5ms
        public Form1()
        {
            InitializeComponent();

            // 시리얼 포트 설정
            serialPort = new SerialPort("COM4", 9600, Parity.None, 8, StopBits.One);
            serialPort.DataReceived += new SerialDataReceivedEventHandler(SerialPort_DataReceived);

            // 송신 버튼 클릭 이벤트 핸들러 등록
            button1.Click += new EventHandler(SendData);  // button1은 송신 버튼입니다.
                                                          // 'Connect' 버튼 클릭 이벤트 핸들러 등록
            ConnectButton.Click += new EventHandler(ConnectButton_Click);  // 'ConnectButton'은 연결 버튼입니다.
                                                                           // 'Disconnect' 버튼 클릭 이벤트 핸들러 등록
            DisconnectButton.Click += new EventHandler(DisconnectButton_Click);  // 'DisconnectButton'은 연결 해제 버튼입니다.
                                                                                 // 마우스 휠 이벤트 핸들러 등록
            chart1.MouseWheel += new MouseEventHandler(Chart1_MouseWheel);
            chart1.ChartAreas[0].AxisX.Title = "시간 (s)";
            chart1.ChartAreas[0].AxisY.Title = "RPM";
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            // 그래프 설정
            chart1.Series.Clear(); // 기존 Series 제거
            chart1.Series.Add("M1"); // 'M1' Series 추가
            chart1.Series.Add("M2"); // 'M2' Series 추가
            chart1.Series["M1"].ChartType = SeriesChartType.Line;
            chart1.Series["M2"].ChartType = SeriesChartType.Line;
            // X축과 Y축의 라벨 설정
           
        }

        // 데이터 수신 이벤트
        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            dataIn = serialPort.ReadLine();
            if (this.IsHandleCreated)  // 컨트롤이 완전히 로드되었는지 확인
            {
                this.Invoke(new EventHandler(ShowData));
            }
        }

        // Connect 버튼 클릭 이벤트
        private void ConnectButton_Click(object sender, EventArgs e)
        {
            if (!serialPort.IsOpen)
            {
                serialPort.Open();  // 시리얼 포트 연결
                stopwatch.Start();  // 데이터 수집 시작
            }
        }

        // Disconnect 버튼 클릭 이벤트
        private void DisconnectButton_Click(object sender, EventArgs e)
        {
            if (serialPort.IsOpen)
            {
                serialPort.Close();  // 시리얼 포트 연결 종료
                stopwatch.Stop();  // 데이터 수집 종료
            }
        }


        // 데이터 표시
        private void ShowData(object sender, EventArgs e)
        {
            // 데이터 파싱
            var data = dataIn.Split(new string[] { "M1: ", "rpm  M2: ", "rpm" }, StringSplitOptions.RemoveEmptyEntries);
            if (data.Length >= 2)
            {
                // M1, M2 값을 찾아서 그래프에 추가
                double m1Value, m2Value;
                if (double.TryParse(data[0], out m1Value) && double.TryParse(data[1], out m2Value))
                {
                    double currentTime = stopwatch.Elapsed.TotalSeconds;
                    chart1.Series["M1"].Points.AddXY(currentTime, m1Value);
                    chart1.Series["M2"].Points.AddXY(currentTime, m2Value);

                    // 수신한 데이터를 텍스트 박스에 표시
                    textBox1.Text = $"M1: {m1Value} RPM, M2: {m2Value} RPM";
                }
            }

            // 스크롤바 설정
            chart1.ChartAreas[0].AxisX.ScrollBar.Enabled = true;
            chart1.ChartAreas[0].AxisX.ScrollBar.IsPositionedInside = true;
            chart1.ChartAreas[0].AxisX.ScrollBar.Size = 10;

            chart1.ChartAreas[0].CursorX.AutoScroll = true;
            chart1.ChartAreas[0].AxisX.ScaleView.Zoomable = true;
            chart1.ChartAreas[0].AxisX.ScaleView.SizeType = DateTimeIntervalType.Number;
            double viewSize = 5.0;  // view size of 20 seconds
            double position = stopwatch.Elapsed.TotalSeconds;
            double start = position > viewSize ? position - viewSize : 0;
            chart1.ChartAreas[0].AxisX.ScaleView.Zoom(start, position);
        }

        private void SendData(object sender, EventArgs e)
        {
            string dataOut = textBox2.Text;  // textBox2에서 데이터를 가져옵니다.
            //string dataOut = "01E";
            serialPort.WriteLine(dataOut);  // 가져온 데이터를 송신합니다.
        }
        private void chart1_Click(object sender, EventArgs e)
        {

        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }
        // 마우스 휠 이벤트 핸들러
        private void Chart1_MouseWheel(object sender, MouseEventArgs e)
        {
            var chart = (Chart)sender;
            var xAxis = chart.ChartAreas[0].AxisX;
            var yAxis = chart.ChartAreas[0].AxisY;

            try
            {
                if (e.Delta < 0) // Zoom out
                {
                    xAxis.ScaleView.ZoomReset();
                    yAxis.ScaleView.ZoomReset();
                }

                if (e.Delta > 0) // Zoom in
                {
                    var xMin = xAxis.ScaleView.ViewMinimum;
                    var xMax = xAxis.ScaleView.ViewMaximum;

                    var yMin = yAxis.ScaleView.ViewMinimum;
                    var yMax = yAxis.ScaleView.ViewMaximum;

                    var posXStart = xAxis.PixelPositionToValue(e.Location.X) - (xMax - xMin) / 4;
                    var posXFinish = xAxis.PixelPositionToValue(e.Location.X) + (xMax - xMin) / 4;

                    var posYStart = yAxis.PixelPositionToValue(e.Location.Y) - (yMax - yMin) / 4;
                    var posYFinish = yAxis.PixelPositionToValue(e.Location.Y) + (yMax - yMin) / 4;

                    xAxis.ScaleView.Zoom(posXStart, posXFinish);
                    yAxis.ScaleView.Zoom(posYStart, posYFinish);
                }
            }
            catch { }
        }
    }
}