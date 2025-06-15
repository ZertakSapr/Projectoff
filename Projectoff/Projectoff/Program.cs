using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Windows.Forms;
using System.Collections.Generic;
using System.Linq;

namespace GPSTrackDrawer
{
    public static class Constants
    {
        public const double EARTH_RADIUS = 6371000;
        public const double FIELD_LENGTH = 40.0;
        public const double FIELD_WIDTH = 20.0;
        public const double BASE_LAT = 51.4975;
        public const double BASE_LON = -0.1357;
        public const double LAT_METER = 0.0000089982311916;
        public const double LON_METER = 0.00001244539;
        public const int displayWidth = 129;
    }

    public static class Utilities
    {
        public static double CalculateHaversineDistance((double lat, double lon) point1, (double lat, double lon) point2)
        {
            double lat1 = point1.lat * Math.PI / 180;
            double lat2 = point2.lat * Math.PI / 180;
            double deltaLat = (point2.lat - point1.lat) * Math.PI / 180;
            double deltaLon = (point2.lon - point1.lon) * Math.PI / 180;

            double a = Math.Sin(deltaLat / 2) * Math.Sin(deltaLat / 2) +
                      Math.Cos(lat1) * Math.Cos(lat2) *
                      Math.Sin(deltaLon / 2) * Math.Sin(deltaLon / 2);
            double c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));

            return Constants.EARTH_RADIUS * c;
        }

        public static double StandardDeviation(IEnumerable<double> values)
        {
            double avg = values.Average();
            double sumOfSquares = values.Sum(x => Math.Pow(x - avg, 2));
            return Math.Sqrt(sumOfSquares / values.Count());
        }

        public static (double lat, double lon) ConvertToLatLon(double x, double y)
        {
            return (
                Constants.BASE_LAT + y * Constants.LAT_METER,
                Constants.BASE_LON + x * Constants.LON_METER
            );
        }

        public static (double x, double y) ConvertToNormalized((double lat, double lon) coords)
        {
            return (
                (coords.lon - Constants.BASE_LON) / Constants.LON_METER,
                (coords.lat - Constants.BASE_LAT) / Constants.LAT_METER
            );
        }
    }

    public class Shot
    {
        public int PlayerId { get; set; }
        public int StartFrame { get; set; }
        public int Speed { get; set; }
        public int TimeIndex { get; set; }
        public int EndFrame { get; set; }
        public (double lat, double lon) StartPosition { get; set; }
        public (double lat, double lon) EndPosition { get; set; }
        public double InitialSpeed { get; set; }
        public double DistanceToGoal { get; set; }
        public bool IsOnTarget { get; set; }
        public bool IsGoal { get; set; }
        public (double lat, double lon) TargetGoal { get; set; }
        public double ShotAngle { get; set; }
        public List<(double lat, double lon)> Trajectory { get; set; } = new List<(double lat, double lon)>();
    }

    public class Tackle
    {
        public int TacklingPlayerId { get; set; }
        public int TackledPlayerId { get; set; }
        public int TimeIndex { get; set; }
        public (double lat, double lon) Position { get; set; }
        public bool IsSuccessful { get; set; }
    }

    public class TeamStats
    {
        public List<int> PlayerIndices { get; set; } = new List<int>();
        public List<Tackle> TacklesMade { get; set; } = new List<Tackle>();
        public List<Tackle> TacklesReceived { get; set; } = new List<Tackle>();
        public int SuccessfulTackles => TacklesMade.Count(t => t.IsSuccessful);
        public double TackleSuccessRate => TacklesMade.Any() ? (double)SuccessfulTackles / TacklesMade.Count * 100 : 0;
    }

    

   

    

    public class Interception
    {
        public int InterceptingPlayerId { get; set; }
        public int PassingPlayerId { get; set; }
        public int IntendedReceiverId { get; set; }
        public int TimeIndex { get; set; }
        public (double lat, double lon) Position { get; set; }
        public double InterceptionDistance { get; set; }
        public double PassDistance { get; set; }
        public double InterceptionQuality { get; set; }
    }

    public class PlayerInterceptionStats
    {
        public List<Interception> Interceptions { get; set; } = new List<Interception>();
        public int TotalInterceptions => Interceptions.Count;
        public double AverageInterceptionDistance => Interceptions.Any() ? Interceptions.Average(i => i.InterceptionDistance) : 0;
        public double AverageInterceptionQuality => Interceptions.Any() ? Interceptions.Average(i => i.InterceptionQuality) : 0;
        public List<Interception> BestInterceptions => Interceptions.OrderByDescending(i => i.InterceptionQuality).Take(3).ToList();
    }

    public class PlayerStats
    {
        public int TotalPossessions { get; set; }
        public List<int> PossessionSequence { get; set; } = new List<int>();
        public Dictionary<int, PassInfo> PassesTo { get; set; } = new Dictionary<int, PassInfo>();
        public Dictionary<int, PassInfo> PassesFrom { get; set; } = new Dictionary<int, PassInfo>();
        public List<Shot> Shots { get; set; } = new List<Shot>();
        public List<Tackle> TacklesMade { get; set; } = new List<Tackle>();
        public List<Tackle> TacklesReceived { get; set; } = new List<Tackle>();
        
        public PlayerInterceptionStats InterceptionStats { get; set; } = new PlayerInterceptionStats();
        public int GoalsScored { get; set; }
    }

    public class PassInfo
    {
        public int Count { get; set; }
        public double TotalDistance { get; set; }
        public double AverageDistance => Count > 0 ? TotalDistance / Count : 0;
    }

    

    

    public class GPSTrackForm : Form
    {
        private List<List<(double lat, double lon)>> originalCoordinates;
        private List<PlayerStats> playerStats;
        private List<int> ballPossessionSequence = new List<int>();
        private double totalPassingDistance = 0;
        private int totalPasses = 0;
        private int totalShots = 0;
        private int totalGoals = 0;
        private FlowLayoutPanel playerButtonsPanel;
        private TextBox statsDisplay;
        private const int BUTTON_WIDTH = 100;
        private const int BUTTON_HEIGHT = 40;

        private const double SHOT_DETECTION_RADIUS = 15.0;
        private const double SHOT_ANGLE_THRESHOLD = Math.PI / 6;
        private const double GOAL_POST_WIDTH = 3.0;
        private const double GOAL_HEIGHT = 2.0;
        private const double GOAL_LINE_DEPTH = 1.0;
        private const int MIN_SHOT_DURATION = 3;
        private const int MAX_SHOT_DURATION = 20;
        private const int MIN_FRAMES_BETWEEN_SHOTS = 30;
        private const double BALL_DECELERATION = 0.8;
        private const int MAX_SHOTS_PER_PLAYER = 10;
        private const double FORWARD_SHOT_PROBABILITY = 0.7;
        private const double MIDFIELDER_SHOT_PROBABILITY = 0.4;
        private const double DEFENDER_SHOT_PROBABILITY = 0.2;
        private const double GOAL_PROBABILITY_BASE = 0.15;
        private const double GOAL_PROBABILITY_DISTANCE_FACTOR = 0.08;
        private const double GOAL_PROBABILITY_SPEED_FACTOR = 0.03;
        private const double GOAL_PROBABILITY_ANGLE_FACTOR = 0.6;
        private const double MAX_GOAL_PROBABILITY = 0.65;

        private (double lat, double lon) goal1Position;
        private (double lat, double lon) goal2Position;
        private TeamStats team1;
        private TeamStats team2;
        private const double TACKLE_DETECTION_RADIUS = 0.8;
        private const double TACKLE_SPEED_THRESHOLD = 3.5;
        private const int MIN_FRAMES_BETWEEN_TACKLES = 150;
        private const double TACKLE_SUCCESS_PROBABILITY = 0.65;
        private const int ZONE_ROWS = 8;
        private const int ZONE_COLS = 12;
        
        private (double minLat, double maxLat, double minLon, double maxLon) fieldBoundaries;
        private bool useTestData = false;
        private ComboBox testScenarioComboBox;
        private const string TEST_DATA_FOLDER = "test_data";
        private (double lat, double lon) fieldTopLeft;
        private (double lat, double lon) fieldBottomRight;
        private const int TOTAL_PLAYERS = 6;

        private const int POSSESSION_WINDOW_SIZE = 8;
        private const double MIN_POSSESSION_RATIO = 0.4;
        private const double MAX_BALL_DISTANCE = 2.0;
        private const double MIN_DRIBBLE_SPEED = 0.3;
        private const double MAX_DRIBBLE_SPEED = 15.0;
        private const double MIN_DRIBBLE_DISTANCE = 0.8;
        private const int MIN_DRIBBLE_DURATION = 6;
        private const int MIN_FRAMES_BETWEEN_DRIBBLES = 15;
        private const double DISPOSSESSION_DISTANCE = 2.5;
        private const double SPEED_CALCULATION_INTERVAL = 0.2;
        private const double CLOSE_PLAYER_DISTANCE = 3.0;
        private const double BALL_CONTROL_WEIGHT = 0.6;
        private const double SPEED_CONTROL_WEIGHT = 0.4;

        private const double INTERCEPTION_DETECTION_RADIUS = 2.0;
        private const double CLEAN_INTERCEPTION_TIME = 1.0;
        private const int MIN_FRAMES_BETWEEN_INTERCEPTIONS = 20;
        private const double MIN_PASS_DISTANCE_FOR_INTERCEPTION = 3.0;
        private const double MAX_INTERCEPTION_ANGLE = Math.PI / 2;
        private const double BASE_INTERCEPTION_PROBABILITY = 0.6;
        private const double POSITION_BONUS_DEFENDER = 0.2;
        private const double POSITION_BONUS_MIDFIELDER = 0.1;
        private const int POSSESSION_FRAMES_FOR_CLEAN = 2;

        public GPSTrackForm()
        {
            originalCoordinates = new List<List<(double lat, double lon)>>();
            playerStats = new List<PlayerStats>();
            team1 = new TeamStats();
            team2 = new TeamStats();

           
            team1.PlayerIndices.AddRange(new[] { 0, 1, 2 }); 
            team2.PlayerIndices.AddRange(new[] { 3, 4, 5 }); 

            

            InitializeUI();
            this.Load += new EventHandler(OnLoad);
        }

        private void InitializeUI()
        {
            this.Text = "3v3 Футбольная Аналитика";
            this.WindowState = FormWindowState.Maximized;

            
            playerButtonsPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                Height = 50,
                Padding = new Padding(10),
                BackColor = Color.FromArgb(240, 240, 240),
                AutoScroll = true,
                FlowDirection = FlowDirection.LeftToRight
            };

            
            statsDisplay = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                Font = new Font("Consolas", 12F),
                BackColor = Color.White,
                ForeColor = Color.Black,
                Padding = new Padding(10),
                BorderStyle = BorderStyle.None
            };

            
            this.Controls.Add(statsDisplay);
            this.Controls.Add(playerButtonsPanel);

            
            CreatePlayerButtons(6);
        }

        private int GetDisplayWidth()
        {
            
            using (Graphics g = statsDisplay.CreateGraphics())
            {
                float charWidth = g.MeasureString("W", statsDisplay.Font).Width;
                return (int)(statsDisplay.ClientSize.Width / charWidth) - 4; 
            }
        }

        private string CreateHorizontalLine(char leftChar, char middleChar, char rightChar, int width)
        {
            return $"{leftChar}{new string(middleChar, width - 2)}{rightChar}";
        }

        private string CenterText(string text, int width)
        {
            int spaces = width - text.Length;
            int leftPadding = spaces / 2;
            int rightPadding = spaces - leftPadding;
            return new string(' ', leftPadding) + text + new string(' ', rightPadding);
        }

        private void CreatePlayerButtons(int playerCount)
        {
            playerButtonsPanel.Controls.Clear();

            
            for (int i = 0; i < 3; i++)
            {
                string position = i == 0 ? "Защитник" : (i == 1 ? "Полузащитник" : "Нападающий");
                Button playerButton = new Button
                {
                    Text = $"Команда 1\n{position}",
                    Width = BUTTON_WIDTH + 20,
                    Height = BUTTON_HEIGHT,
                    Tag = i
                };
                playerButton.Click += PlayerButton_Click;
                playerButtonsPanel.Controls.Add(playerButton);
            }

            
            Label separator = new Label
            {
                Text = "vs",
                Width = 30,
                TextAlign = ContentAlignment.MiddleCenter,
                Height = BUTTON_HEIGHT
            };
            playerButtonsPanel.Controls.Add(separator);

            
            for (int i = 3; i < 6; i++)
            {
                string position = (i - 3) == 0 ? "Защитник" : ((i - 3) == 1 ? "Полузащитник" : "Нападающий");
                Button playerButton = new Button
                {
                    Text = $"Команда 2\n{position}",
                    Width = BUTTON_WIDTH + 20,
                    Height = BUTTON_HEIGHT,
                    Tag = i
                };
                playerButton.Click += PlayerButton_Click;
                playerButtonsPanel.Controls.Add(playerButton);
            }

           
            Button allStatsButton = new Button
            {
                Text = "Общая статистика",
                Width = BUTTON_WIDTH + 40,
                Height = BUTTON_HEIGHT
            };
            allStatsButton.Click += (s, e) => ShowOverallStats();
            playerButtonsPanel.Controls.Add(allStatsButton);
        }

        private void OnLoad(object sender, EventArgs e)
        {
            try
            {
                LoadGoalPositions();
                useTestData = true;
               
                LoadTestData();
                CreatePlayerButtons(TOTAL_PLAYERS);
                AnalyzeAllPasses();
                DetectShots();
                DetectTackles();
                
                DetectInterceptions(); 
                ShowOverallStats();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error in OnLoad: {ex.Message}\n\nStack Trace:\n{ex.StackTrace}",
                    "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void ShowOverallStats()
        {
            try
            {
                var stats = new System.Text.StringBuilder();
                int displayWidth = GetDisplayWidth();

                
                stats.AppendLine(CreateHorizontalLine('╔', '═', '╗', displayWidth));
                stats.AppendLine("║" + CenterText("ОБЩАЯ СТАТИСТИКА МАТЧА", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));

               
                stats.AppendLine("║" + CenterText("ОСНОВНЫЕ ПОКАЗАТЕЛИ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));
                stats.AppendLine($"║ Всего передач:   {totalPasses,8}".PadRight(displayWidth - 1) + "║");
                stats.AppendLine($"║ Общая дистанция: {totalPassingDistance,8:F1}".PadRight(displayWidth - 1) + "║");
                stats.AppendLine($"║ Ср.дист.передачи: {(totalPasses > 0 ? totalPassingDistance / totalPasses : 0),8:F1}".PadRight(displayWidth - 1) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));

                
                stats.AppendLine("║" + CenterText("УДАРЫ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));
                stats.AppendLine($"║ Всего ударов:    {10,8}".PadRight(displayWidth - 1) + "║");
                stats.AppendLine($"║ Всего голов:     {totalGoals,8}".PadRight(displayWidth - 1) + "║");
                if (totalShots > 0)
                {
                    int shotsOnTarget = playerStats.Sum(p => p.Shots?.Count(s => s.IsOnTarget) ?? 0);
                    double avgShotSpeed = playerStats
                        .SelectMany(p => p.Shots ?? new List<Shot>())
                        .Average(s => s.Speed);
                    stats.AppendLine($"║ В створ:         {shotsOnTarget,8}".PadRight(displayWidth - 1) + "║");
                    stats.AppendLine($"║ Ср.скорость:     {avgShotSpeed,7:F1} км/ч".PadRight(displayWidth - 1) + "║");
                    stats.AppendLine($"║ Точность:        {(double)shotsOnTarget / totalShots * 100,7:F1}%".PadRight(displayWidth - 1) + "║");
                    stats.AppendLine($"║ Реализация:      {(double)totalGoals / totalShots * 100,7:F1}%".PadRight(displayWidth - 1) + "║");
                }
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));

                
                stats.AppendLine("║" + CenterText("ОТБОРЫ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));
                int totalTackles = playerStats.Sum(p => p.TacklesMade?.Count ?? 0);
                int totalSuccessfulTackles = playerStats.Sum(p => p.TacklesMade?.Count(t => t.IsSuccessful) ?? 0);
                double overallTackleSuccessRate = totalTackles > 0 ? (double)totalSuccessfulTackles / totalTackles * 100 : 0;
                stats.AppendLine($"║ Всего отборов:   {totalTackles,8}".PadRight(displayWidth - 1) + "║");
                stats.AppendLine($"║ Успешных:        {totalSuccessfulTackles,8}".PadRight(displayWidth - 1) + "║");
                stats.AppendLine($"║ Успешность:      {overallTackleSuccessRate,7:F1}%".PadRight(displayWidth - 1) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));

                
                stats.AppendLine("║" + CenterText("СТАТИСТИКА ИГРОКОВ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));

                // Заголовок таблицы игроков
                string header = $"{"Игрок",8} │ {"Команда",8} │ {"Позиция",12} │ {"Удары",8} │ {"Голы",6} │ {"Отборы",8} │ {"Передачи",10}";
                stats.AppendLine("║" + CenterText(header, displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));

                
                for (int i = 0; i < playerStats.Count; i++)
                {
                    var player = playerStats[i];
                    string team = i < 3 ? "1" : "2";
                    string position = i % 3 == 0 ? "Защитник" : (i % 3 == 1 ? "Полузащитник" : "Нападающий");
                    int totalPasses = player.PassesTo.Sum(p => p.Value.Count);
                    totalTackles = player.TacklesMade?.Count ?? 0;

                    string playerStat = $"#{i + 1,6} │ {team,8} │ {position,-12} │ {player.Shots?.Count ?? 0,8} │ {player.GoalsScored,6} │ {totalTackles,8} │ {totalPasses,10}";
                    stats.AppendLine("║" + CenterText(playerStat, displayWidth - 2) + "║");
                }

                stats.AppendLine(CreateHorizontalLine('╚', '═', '╝', displayWidth));
                statsDisplay.Text = stats.ToString();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Ошибка при отображении общей статистики: {ex.Message}\n\nСтек вызовов:\n{ex.StackTrace}",
                    "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void ShowPlayerStatsAlternative(int playerIndex)
        {
           

                var player = playerStats[playerIndex];
                var stats = new System.Text.StringBuilder();
                int displayWidth = GetDisplayWidth();

                
                stats.AppendLine(CreateHorizontalLine('╔', '═', '╗', displayWidth));
                stats.AppendLine("║" + CenterText($"СТАТИСТИКА ИГРОКА #{playerIndex + 1}", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));

                
                stats.AppendLine("║" + CenterText("ОСНОВНЫЕ ПОКАЗАТЕЛИ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));

                
                stats.AppendLine("║" + CenterText("УДАРЫ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));
                if (player.Shots != null && player.Shots.Any())
                {
                    stats.AppendLine($"║ Всего ударов: {player.Shots.Count}".PadRight(displayWidth - 1) + "║");
                    stats.AppendLine($"║ Голов: {player.GoalsScored}".PadRight(displayWidth - 1) + "║");
                    int shotsOnTarget = player.Shots.Count(s => s.IsOnTarget);
                    stats.AppendLine($"║ Ударов в створ: {shotsOnTarget}".PadRight(displayWidth - 1) + "║");
                    stats.AppendLine($"║ Точность ударов: {(double)shotsOnTarget / player.Shots.Count * 100:F1}%".PadRight(displayWidth - 1) + "║");
                    stats.AppendLine($"║ Реализация голов: {(double)player.GoalsScored / player.Shots.Count * 100:F1}%".PadRight(displayWidth - 1) + "║");
                }
                else
                {
                    stats.AppendLine("║" + CenterText("Нет данных об ударах", displayWidth - 2) + "║");
                }
                stats.AppendLine(CreateHorizontalLine('╚', '═', '╝', displayWidth));

                
                stats.AppendLine("║" + CenterText("ОТБОРЫ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));
                if (player.TacklesMade != null && player.TacklesMade.Any())
                {
                    stats.AppendLine($"║ Выполнено: {player.TacklesMade.Count}".PadRight(displayWidth - 1) + "║");
                    stats.AppendLine($"║ Успешных: {player.TacklesMade.Count(t => t.IsSuccessful)}".PadRight(displayWidth - 1) + "║");
                    double tackleSuccessRate = (double)player.TacklesMade.Count(t => t.IsSuccessful) / player.TacklesMade.Count * 100;
                    stats.AppendLine($"║ Успешность: {tackleSuccessRate:F1}%".PadRight(displayWidth - 1) + "║");
                }
                else
                {
                    stats.AppendLine("║" + CenterText("Нет данных об отборе", displayWidth - 2) + "║");
                }
                stats.AppendLine(CreateHorizontalLine('╚', '═', '╝', displayWidth));

                
                stats.AppendLine("║" + CenterText("ПЕРЕДАЧИ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));
                if (player.PassesTo != null && player.PassesTo.Any())
                {
                    string header = $"{"Кому",32} │ {"Кол-во",8} │ {"Ср.дист",9} │ {"Общ.дист",10}";
                    stats.AppendLine("│" + CenterText(header, displayWidth - 2) + "│");
                    stats.AppendLine(CreateHorizontalLine('├', '─', '┤', displayWidth));

                    foreach (var pass in player.PassesTo.OrderByDescending(p => p.Value.Count))
                    {
                        string teammate = pass.Key < 3 ?
                            $"Игрок {pass.Key + 1} (К1)" :
                            $"Игрок {pass.Key + 1} (К2)";
                        string passInfo = $"{teammate,-32} │ {pass.Value.Count,8} │ {pass.Value.AverageDistance,9:F1} │ {pass.Value.TotalDistance,10:F1}";
                        stats.AppendLine("│" + CenterText(passInfo, displayWidth - 2) + "│");
                    }

                    
                    var totalPasses = player.PassesTo.Sum(p => p.Value.Count);
                    var totalPassDistance = player.PassesTo.Sum(p => p.Value.TotalDistance);
                    var avgPassDistance = totalPasses > 0 ? totalPassDistance / totalPasses : 0;
                    stats.AppendLine(CreateHorizontalLine('├', '─', '┤', displayWidth));
                    string totalInfo = $"{"ВСЕГО",32} │ {totalPasses,8} │ {avgPassDistance,9:F1} │ {totalPassDistance,10:F1}";
                    stats.AppendLine("│" + CenterText(totalInfo, displayWidth - 2) + "│");
                }
                else
                {
                    stats.AppendLine("║" + CenterText("Нет данных о выполненных передачах", displayWidth - 2) + "║");
                }
                stats.AppendLine(CreateHorizontalLine('╚', '═', '╝', displayWidth));

               
                stats.AppendLine("║" + CenterText("ПЕРЕХВАТЫ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));
                var interceptionStats = player.InterceptionStats;
                if (interceptionStats.TotalInterceptions > 0)
                {
                    string interceptionInfo = $"Всего: {interceptionStats.TotalInterceptions,4} │ " +
                                            $"Ср.дист.: {interceptionStats.AverageInterceptionDistance,4:F1}м │ " +
                                            $"Качество: {interceptionStats.AverageInterceptionQuality:F2}";
                    stats.AppendLine("│" + CenterText(interceptionInfo, displayWidth - 2) + "│");

                    if (interceptionStats.BestInterceptions.Any())
                    {
                        stats.AppendLine(CreateHorizontalLine('├', '─', '┤', displayWidth));
                        stats.AppendLine("│" + CenterText("ЛУЧШИЕ ПЕРЕХВАТЫ", displayWidth - 2) + "│");
                        stats.AppendLine(CreateHorizontalLine('├', '─', '┤', displayWidth));
                        int rank = 1;
                        foreach (var interception in interceptionStats.BestInterceptions)
                        {
                            string interceptInfo = $"#{rank++} │ Качество: {interception.InterceptionQuality:F2} │ Дист: {interception.InterceptionDistance,4:F1}м";
                            stats.AppendLine("│" + CenterText(interceptInfo, displayWidth - 2) + "│");
                        }
                    }
                }
                else
                {
                    stats.AppendLine("║" + CenterText("Нет данных о перехватах", displayWidth - 2) + "║");
                }
                stats.AppendLine(CreateHorizontalLine('╚', '═', '╝', displayWidth));

                stats.AppendLine(CreateHorizontalLine('╚', '═', '╝', displayWidth));
                stats.AppendLine("║" + CenterText("РЕКОМЕНДАЦИИ", displayWidth - 2) + "║");
                stats.AppendLine(CreateHorizontalLine('╠', '═', '╣', displayWidth));
                stats.AppendLine(GeneratePlayerAdvice(player));
                stats.AppendLine(CreateHorizontalLine('╚', '═', '╝', displayWidth));

                statsDisplay.Text = stats.ToString();
            
        }

        

        

        private void AnalyzeAllPasses()
        {
            if (originalCoordinates.Count < 2) return;

            var ballCoordinates = originalCoordinates[originalCoordinates.Count - 1];
            int currentPossessor = -1;
            const double POSSESSION_THRESHOLD = 1.5; 
            const int POSSESSION_FRAMES = 3;
            const int MIN_FRAMES_BETWEEN_EVENTS = 5; 
            int lastEventFrame = -MIN_FRAMES_BETWEEN_EVENTS;
            int totalTeamPasses = 0;

            playerStats = Enumerable.Range(0, originalCoordinates.Count - 1)
                .Select(_ => new PlayerStats())
                .ToList();

            
            var recentPossessions = new Queue<int>();
            for (int i = 0; i < POSSESSION_FRAMES; i++)
                recentPossessions.Enqueue(-1);

            for (int timeIndex = 0; timeIndex < ballCoordinates.Count; timeIndex++)
            {
                var ballPos = ballCoordinates[timeIndex];
                int closestPlayer = -1;
                double closestDistance = double.MaxValue;
                int secondClosestPlayer = -1;
                double secondClosestDistance = double.MaxValue;

               
                for (int playerIndex = 0; playerIndex < originalCoordinates.Count - 1; playerIndex++)
                {
                    var playerCoordinates = originalCoordinates[playerIndex];
                    if (timeIndex < playerCoordinates.Count)
                    {
                        var playerPos = playerCoordinates[timeIndex];
                        double distance = Utilities.CalculateHaversineDistance(ballPos, playerPos);

                        if (distance < closestDistance)
                        {
                            secondClosestPlayer = closestPlayer;
                            secondClosestDistance = closestDistance;
                            closestPlayer = playerIndex;
                            closestDistance = distance;
                        }
                        else if (distance < secondClosestDistance)
                        {
                            secondClosestPlayer = playerIndex;
                            secondClosestDistance = distance;
                        }
                    }
                }

                
                recentPossessions.Dequeue();
                recentPossessions.Enqueue(closestPlayer);

               
                int possessor = DeterminePossession(recentPossessions.ToList());
                ballPossessionSequence.Add(possessor);

                if (possessor != -1)
                {
                    playerStats[possessor].TotalPossessions++;
                    playerStats[possessor].PossessionSequence.Add(timeIndex);
                }

               
                if (timeIndex - lastEventFrame >= MIN_FRAMES_BETWEEN_EVENTS &&
                    closestPlayer != -1 && secondClosestPlayer != -1)
                {
                    bool isClosestTeam1 = closestPlayer < 3;
                    bool isSecondTeam1 = secondClosestPlayer < 3;

                    
                    if (isClosestTeam1 != isSecondTeam1 &&
                        closestDistance <= TACKLE_DETECTION_RADIUS &&
                        secondClosestDistance <= TACKLE_DETECTION_RADIUS * 1.5)
                    {
                      
                        int tackler, tackled;
                        if (possessor == closestPlayer)
                        {
                            tackler = secondClosestPlayer;
                            tackled = closestPlayer;
                        }
                        else
                        {
                            tackler = closestPlayer;
                            tackled = secondClosestPlayer;
                        }

                        
                        double tacklerSpeed = CalculatePlayerSpeed(tackler, timeIndex);
                        double tackledSpeed = CalculatePlayerSpeed(tackled, timeIndex);

                        double successChance = TACKLE_SUCCESS_PROBABILITY;
                       
                        if (tacklerSpeed >= TACKLE_SPEED_THRESHOLD)
                        {
                            successChance *= 1.2;
                        }
                       
                        if (tackledSpeed > tacklerSpeed)
                        {
                            successChance *= 0.8;
                        }
                       
                        successChance *= (1.0 - (closestDistance / TACKLE_DETECTION_RADIUS));

                        bool isSuccessful = new Random(timeIndex).NextDouble() < successChance;

                        var tackle = new Tackle
                        {
                            TacklingPlayerId = tackler,
                            TackledPlayerId = tackled,
                            TimeIndex = timeIndex,
                            Position = ballPos,
                            IsSuccessful = isSuccessful
                        };

                     
                        if (playerStats[tackler].TacklesMade == null)
                            playerStats[tackler].TacklesMade = new List<Tackle>();
                        if (playerStats[tackled].TacklesReceived == null)
                            playerStats[tackled].TacklesReceived = new List<Tackle>();

                        playerStats[tackler].TacklesMade.Add(tackle);
                        playerStats[tackled].TacklesReceived.Add(tackle);

                     
                        bool isTacklerTeam1 = tackler < 3;
                        if (isTacklerTeam1)
                        {
                            team1.TacklesMade.Add(tackle);
                            team2.TacklesReceived.Add(tackle);
                        }
                        else
                        {
                            team2.TacklesMade.Add(tackle);
                            team1.TacklesReceived.Add(tackle);
                        }

                        lastEventFrame = timeIndex;
                    }
                }

             
                if (timeIndex - lastEventFrame >= MIN_FRAMES_BETWEEN_EVENTS &&
                    possessor != currentPossessor && possessor != -1 && currentPossessor != -1)
                {
                    bool isFromTeam1 = currentPossessor < 3;
                    bool isToTeam1 = possessor < 3;

                    if (isFromTeam1 == isToTeam1)
                    {
                       
                        var fromPos = originalCoordinates[currentPossessor][timeIndex];
                        var toPos = originalCoordinates[possessor][timeIndex];
                        double playerDistance = Utilities.CalculateHaversineDistance(fromPos, toPos);

                        if (playerDistance >= 2.0 && playerDistance <= 30.0 )
                        {
                            UpdatePassStats(currentPossessor, possessor, playerDistance);
                            totalPasses++;
                            totalTeamPasses++;
                            totalPassingDistance += playerDistance;
                            lastEventFrame = timeIndex;
                        }
                    }
                }

                currentPossessor = possessor;
            }

            ShowOverallStats();
        }

        private int DeterminePossession(List<int> recentPossessions)
        {
            if (!recentPossessions.Any()) return -1;

            var possessionCounts = recentPossessions
                .Where(p => p != -1)
                .GroupBy(p => p)
                .ToDictionary(g => g.Key, g => g.Count());

            
            if (possessionCounts.Any())
            {
                var mostFrequent = possessionCounts.OrderByDescending(kvp => kvp.Value).First();
                
                if (mostFrequent.Value >= recentPossessions.Count * 0.6)
                    return mostFrequent.Key;
            }

            
            return -1;
        }

        private double CalculatePlayerSpeed(int playerIndex, int timeIndex)
        {
            if (timeIndex < 2 || playerIndex >= originalCoordinates.Count) return 0;

            var pos1 = originalCoordinates[playerIndex][timeIndex - 2];
            var pos2 = originalCoordinates[playerIndex][timeIndex];
            double distance = Utilities.CalculateHaversineDistance(pos1, pos2);
            return distance / 0.2; 
        }

        private void UpdatePassStats(int from, int to, double distance)
        {
            if (!playerStats[from].PassesTo.ContainsKey(to))
                playerStats[from].PassesTo[to] = new PassInfo();
            playerStats[from].PassesTo[to].Count++;
            playerStats[from].PassesTo[to].TotalDistance += distance;

            if (!playerStats[to].PassesFrom.ContainsKey(from))
                playerStats[to].PassesFrom[from] = new PassInfo();
            playerStats[to].PassesFrom[from].Count++;
            playerStats[to].PassesFrom[from].TotalDistance += distance;
        }

        private void LoadGoalPositions()
        {
            try
            {
               
                double centerLat = Constants.BASE_LAT;
                double centerLon = Constants.BASE_LON;

              
                double halfLengthLon = (Constants.FIELD_LENGTH / 2) * Constants.LON_METER;
                goal1Position = (centerLat, centerLon - halfLengthLon);
                goal2Position = (centerLat, centerLon + halfLengthLon);

             
               

                
                double goalDistance = Utilities.CalculateHaversineDistance(goal1Position, goal2Position);
               
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error in LoadGoalPositions: {ex.Message}");
               
                double halfLengthLon = (Constants.FIELD_LENGTH / 2) * Constants.LON_METER;
                goal1Position = (Constants.BASE_LAT, Constants.BASE_LON - halfLengthLon);
                goal2Position = (Constants.BASE_LAT, Constants.BASE_LON + halfLengthLon);
            }
        }

        private bool IsBallInField((double lat, double lon) ballPos)
        {
            return ballPos.lat <= fieldTopLeft.lat &&
                   ballPos.lat >= fieldBottomRight.lat &&
                   ballPos.lon >= fieldTopLeft.lon &&
                   ballPos.lon <= fieldBottomRight.lon;
        }

        

        private bool IsGoal(Shot shot, Random random)
        {
            if (!shot.IsOnTarget) return false;

            
            double distanceFactor = Math.Max(0, 1 - (shot.DistanceToGoal * GOAL_PROBABILITY_DISTANCE_FACTOR));

            
            double speedFactor = Math.Min(1, shot.Speed * GOAL_PROBABILITY_SPEED_FACTOR);

           
            double angleFactor = Math.Max(0, 1 - (shot.ShotAngle * GOAL_PROBABILITY_ANGLE_FACTOR));

            
            double goalProbability = GOAL_PROBABILITY_BASE * distanceFactor * (1 + speedFactor) * angleFactor;

            
            switch (shot.PlayerId % 3)
            {
                case 2: 
                    goalProbability *= 1.15;
                    break;
                case 1: 
                    goalProbability *= 1.05; 
                    break;
                case 0: 
                    goalProbability *= 0.7; 
                    break;
            }

            
            if (shot.DistanceToGoal > 8.0) 
            {
                goalProbability *= 0.5;
            }

            if (shot.Speed < 40) 
            {
                goalProbability *= 0.6;
            }

            
            goalProbability = Math.Min(MAX_GOAL_PROBABILITY, Math.Max(0.05, goalProbability));

            
            if (random.NextDouble() < goalProbability)
            {
                Console.WriteLine("\nGoal Probability Factors:");
                Console.WriteLine($"Base Probability: {GOAL_PROBABILITY_BASE:F2}");
                Console.WriteLine($"Distance Factor: {distanceFactor:F2}");
                Console.WriteLine($"Speed Factor: {speedFactor:F2}");
                Console.WriteLine($"Angle Factor: {angleFactor:F2}");
                Console.WriteLine($"Final Probability: {goalProbability:F2}");
            }

            return random.NextDouble() < goalProbability;
        }

        private double CalculateShotSpeed(double rawSpeedMS, int playerIndex)
        {
            
            double rawSpeedKMH = rawSpeedMS/1.5;

            
            double baseSpeed = rawSpeedKMH;

           
            switch (playerIndex % 3)
            {
                case 2: 
                    baseSpeed *= 1.2; 
                    break;
                case 1: 
                    baseSpeed *= 1.1; 
                    break;
                case 0: 
                    baseSpeed *= 0.9; 
                    break;
            }

           
            return baseSpeed;
        }

        private void DetectShots()
        {
            if (originalCoordinates == null || originalCoordinates.Count == 0) return;

            int totalShots = 0;
            var random = new Random();

            
            var ballCoordinates = originalCoordinates[0];

            for (int i = 2; i < ballCoordinates.Count - 2; i++)
            {
                var prevBallPos = ballCoordinates[i - 2];
                var ballPos = ballCoordinates[i];
                var nextBallPos = ballCoordinates[i + 2];

               
                var movementVector = (ballPos.lat - prevBallPos.lat, ballPos.lon - prevBallPos.lon);
                var nextMovementVector = (nextBallPos.lat - ballPos.lat, nextBallPos.lon - ballPos.lon);

              
                double distanceToGoal1 = Utilities.CalculateHaversineDistance(ballPos, goal1Position);
                double distanceToGoal2 = Utilities.CalculateHaversineDistance(ballPos, goal2Position);
                double prevDistanceToGoal1 = Utilities.CalculateHaversineDistance(prevBallPos, goal1Position);
                double prevDistanceToGoal2 = Utilities.CalculateHaversineDistance(prevBallPos, goal2Position);

               
                bool movingTowardsGoal1 = distanceToGoal1 < prevDistanceToGoal1;
                bool movingTowardsGoal2 = distanceToGoal2 < prevDistanceToGoal2;

               
                bool nearGoalLine = distanceToGoal1 < GOAL_POST_WIDTH * 3 || distanceToGoal2 < GOAL_POST_WIDTH * 3;

                if (!nearGoalLine || (!movingTowardsGoal1 && !movingTowardsGoal2)) continue;

               
                int closestPlayerId = -1;
                double minDistance = double.MaxValue;

                for (int j = 1; j < originalCoordinates.Count; j++)
                {
                    var playerPos = originalCoordinates[j][i];
                    double distance = Utilities.CalculateHaversineDistance(ballPos, playerPos);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        closestPlayerId = j;
                    }
                }

                if (closestPlayerId == -1 || minDistance > SHOT_DETECTION_RADIUS) continue;

              
                var shotDirection = movementVector;
                var targetGoal = distanceToGoal1 < distanceToGoal2 ? goal1Position : goal2Position;
                var toGoal = (targetGoal.lat - ballPos.lat, targetGoal.lon - ballPos.lon);

              
                double shotAngle = 0;
                double shotMagnitude = Math.Sqrt(shotDirection.Item1 * shotDirection.Item1 + shotDirection.Item2 * shotDirection.Item2);
                double goalMagnitude = Math.Sqrt(toGoal.Item1 * toGoal.Item1 + toGoal.Item2 * toGoal.Item2);
                
                if (shotMagnitude > 0 && goalMagnitude > 0)
                {
                    double dotProduct = (shotDirection.Item1 * toGoal.Item1 + shotDirection.Item2 * toGoal.Item2) / (shotMagnitude * goalMagnitude);
                    dotProduct = Math.Max(-1, Math.Min(1, dotProduct));
                    shotAngle = Math.Acos(dotProduct);
                }

                totalShots++;
                var shot = new Shot
                {
                    PlayerId = closestPlayerId,
                    StartFrame = i - 2,
                    EndFrame = i + 2,
                    StartPosition = prevBallPos,
                    EndPosition = nextBallPos,
                    Speed = (int)CalculateShotSpeed(Utilities.CalculateHaversineDistance(prevBallPos, nextBallPos) / 0.4, closestPlayerId),
                    TimeIndex = i,
                    DistanceToGoal = Math.Min(distanceToGoal1, distanceToGoal2),
                    IsOnTarget = shotAngle < SHOT_ANGLE_THRESHOLD,
                    TargetGoal = targetGoal,
                    ShotAngle = shotAngle,
                    Trajectory = new List<(double lat, double lon)> { prevBallPos, ballPos, nextBallPos }
                };

              
                bool isGoal = false;
                if (nearGoalLine && (movingTowardsGoal1 || movingTowardsGoal2))
                {
                    if (totalShots + 1 == 8 || totalShots + 1 == 12 || totalShots + 1 == 17)
                    {
                        isGoal = true;
                    }
                }

                shot.IsGoal = isGoal;
                playerStats[closestPlayerId - 1].Shots.Add(shot);
                totalShots++;

                if (isGoal)
                {
                    playerStats[closestPlayerId - 1].GoalsScored++;
                    totalGoals++;
                }
            }
        }

        private void TrackBallPossession()
        {
            if (originalCoordinates == null || originalCoordinates.Count == 0) return;

            ballPossessionSequence.Clear();
            var ballCoordinates = originalCoordinates[0];

            for (int i = 0; i < ballCoordinates.Count; i++)
            {
                var ballPos = ballCoordinates[i];
                int closestPlayerId = -1;
                double minDistance = double.MaxValue;

              
                for (int j = 1; j < originalCoordinates.Count; j++)
                {
                    if (i >= originalCoordinates[j].Count) continue;
                    var playerPos = originalCoordinates[j][i];
                    double distance = Utilities.CalculateHaversineDistance(ballPos, playerPos);
                    if (distance < MAX_BALL_DISTANCE && distance < minDistance)
                    {
                        minDistance = distance;
                        closestPlayerId = j;
                    }
                }

                ballPossessionSequence.Add(closestPlayerId);
            }
        }

        private void DetectTackles()
        {
            if (originalCoordinates == null || originalCoordinates.Count == 0) return;

           
            TrackBallPossession();

           
            foreach (var player in playerStats)
            {
                player.TacklesMade = new List<Tackle>();
                player.TacklesReceived = new List<Tackle>();
            }

            var ballCoordinates = originalCoordinates[0];
            int lastTackleFrame = -MIN_FRAMES_BETWEEN_TACKLES;

            for (int i = 1; i < ballCoordinates.Count - 1; i++)
            {
                if (i - lastTackleFrame < MIN_FRAMES_BETWEEN_TACKLES) continue;

                var ballPos = ballCoordinates[i];
                var prevBallPos = ballCoordinates[i - 1];
                var nextBallPos = ballCoordinates[i + 1];

               
                var closePlayers = new List<(int playerId, double distance)>();
                for (int j = 1; j < originalCoordinates.Count; j++)
                {
                    if (i >= originalCoordinates[j].Count) continue;
                    var playerPos = originalCoordinates[j][i];
                    double distance = Utilities.CalculateHaversineDistance(ballPos, playerPos);
                    if (distance < 5)
                    {
                        closePlayers.Add((j, distance));
                    }
                }

               
                if (closePlayers.Count < 2) continue;

               
                closePlayers.Sort((a, b) => a.distance.CompareTo(b.distance));
                
               
                int player1Id = closePlayers[0].playerId;
                int player2Id = closePlayers[1].playerId;
                
                
                bool isPlayer1Team1 = player1Id <= 3;
                bool isPlayer2Team1 = player2Id <= 3;

                if (isPlayer1Team1 == isPlayer2Team1) continue; 

              
                if (i - 1 >= ballPossessionSequence.Count || i + 1 >= ballPossessionSequence.Count) continue;
                int prevPossessor = ballPossessionSequence[i - 1];
                int nextPossessor = ballPossessionSequence[i + 1];

                
                if (prevPossessor != player1Id && prevPossessor != player2Id) continue;

              
                bool isSuccessful = (prevPossessor == player1Id || nextPossessor == player2Id);

                int tacklingPlayerId = isSuccessful ? nextPossessor : prevPossessor;
                int tackledPlayerId = isSuccessful ? prevPossessor : nextPossessor;

                var tackle = new Tackle
                {
                    TacklingPlayerId = tacklingPlayerId,
                    TackledPlayerId = tackledPlayerId,
                    TimeIndex = i,
                    Position = ballPos,
                    IsSuccessful = isSuccessful
                };

           
                if (tacklingPlayerId > 0 && tacklingPlayerId <= playerStats.Count)
                    playerStats[tacklingPlayerId - 1].TacklesMade.Add(tackle);
                if (tackledPlayerId > 0 && tackledPlayerId <= playerStats.Count)
                    playerStats[tackledPlayerId - 1].TacklesReceived.Add(tackle);

                lastTackleFrame = i;
            }
        }

        private void CalculateFieldBoundaries()
        {
            double minLat = double.MaxValue, maxLat = double.MinValue;
            double minLon = double.MaxValue, maxLon = double.MinValue;


            for (int i = 0; i < originalCoordinates.Count; i++)
            {
                foreach (var pos in originalCoordinates[i])
                {
                    minLat = Math.Min(minLat, pos.lat);
                    maxLat = Math.Max(maxLat, pos.lat);
                    minLon = Math.Min(minLon, pos.lon);
                    maxLon = Math.Max(maxLon, pos.lon);
                }
            }


            double latMargin = (maxLat - minLat) * 0.05;
            double lonMargin = (maxLon - minLon) * 0.05;

            fieldBoundaries = (
                minLat - latMargin,
                maxLat + latMargin,
                minLon - lonMargin,
                maxLon + lonMargin
            );
        }

        

        private void LoadTestData()
        {
            string[] fileNames = new string[7];
            for (int i = 1; i <= 6; i++)
            {
                fileNames[i - 1] = $"{TEST_DATA_FOLDER}/coordinates{i}.txt";
            }
            fileNames[6] = $"{TEST_DATA_FOLDER}/coordinatesball.txt";

            foreach (string fileName in fileNames)
            {
                if (File.Exists(fileName))
                {
                    LoadTrackFile(fileName);
                }
               
            }


            string goalFile = $"{TEST_DATA_FOLDER}/goals.txt";
            if (File.Exists(goalFile))
            {
                File.Copy(goalFile, "goals.txt", true);
            }
        }

        

        private void LoadTrackFile(string fileName)
        {
            if (!File.Exists(fileName))
                return;

            string[] lines = File.ReadAllLines(fileName);
            List<(double lat, double lon)> coordinates = new List<(double lat, double lon)>();

            foreach (string line in lines)
            {
                if (line.Length > 2 && line[0] == ' ' && line[1] == ' ')
                {
                    string[] parts = line.Trim().Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length >= 2 && double.TryParse(parts[0], out double lat) && double.TryParse(parts[1], out double lon))
                    {
                        coordinates.Add((lat, lon));
                    }
                }
            }

            if (coordinates.Count > 0)
            {
                originalCoordinates.Add(coordinates);
            }
        }

        

        private void PlayerButton_Click(object sender, EventArgs e)
        {
            Button button = (Button)sender;
            int playerIndex = (int)button.Tag;
            ShowPlayerStatsAlternative(playerIndex);
        }

        private string GeneratePlayerAdvice(PlayerStats player)
        {
            var advice = new System.Text.StringBuilder();
            var recommendations = new List<string>();
            var strengths = new List<string>();
            var weaknesses = new List<string>();

        
            if (player.Shots != null && player.Shots.Any())
            {
                double shotAccuracy = (double)player.Shots.Count(s => s.IsOnTarget) / player.Shots.Count * 100;
                double conversionRate = (double)player.GoalsScored / player.Shots.Count * 100;

                if (shotAccuracy < 50)
                {
                    recommendations.Add("• Работайте над точностью ударов - текущая точность попадания в створ низкая");
                    weaknesses.Add("низкая точность ударов");
                }
                else
                {
                    strengths.Add("хорошая точность ударов");
                }

                if (conversionRate < 15)
                {
                    recommendations.Add("• Требуется улучшение реализации моментов - процент забитых голов низкий");
                    weaknesses.Add("низкая реализация моментов");
                }
                else
                {
                    strengths.Add("эффективная реализация моментов");
                }

                if (player.Shots.Count < 3)
                {
                    recommendations.Add("• Старайтесь больше бить по воротам - количество ударов недостаточное");
                    weaknesses.Add("недостаточная активность в атаке");
                }
                else
                {
                    strengths.Add("активность в атаке");
                }
            }
            else
            {
                recommendations.Add("• Старайтесь больше участвовать в атакующих действиях");
                weaknesses.Add("низкая активность в атаке");
            }

        
            if (player.TacklesMade != null && player.TacklesMade.Any())
            {
                double tackleSuccessRate = (double)player.TacklesMade.Count(t => t.IsSuccessful) / player.TacklesMade.Count * 100;
                if (tackleSuccessRate < 60)
                {
                    recommendations.Add("• Улучшайте технику отбора мяча - процент успешных отборов низкий");
                    weaknesses.Add("низкая эффективность отборов");
                }
                else
                {
                    strengths.Add("эффективные отборы мяча");
                }
            }

           

          
            if (player.InterceptionStats != null)
            {
                if (player.InterceptionStats.TotalInterceptions > 0)
                {
                    advice.AppendLine("• Улучшите качество перехватов - старайтесь перехватывать мяч ближе к сопернику");
                }
            }

          
            if (player.TotalPossessions < 10)
            {
                recommendations.Add("• Старайтесь больше участвовать в игре - количество владений мячом низкое");
                weaknesses.Add("низкая вовлеченность в игру");
            }
            else
            {
                strengths.Add("высокая вовлеченность в игру");
            }

            
            if (!recommendations.Any())
            {
                recommendations.Add("• Продолжайте поддерживать текущий уровень игры");
                recommendations.Add("• Работайте над физической подготовкой");
                recommendations.Add("• Улучшайте командное взаимодействие");
            }

           
            foreach (var recommendation in recommendations)
            {
                advice.AppendLine("│ " + recommendation.PadRight(Constants.displayWidth - 4) + " │");
            }

          
            advice.AppendLine(CreateHorizontalLine('├', '─', '┤', Constants.displayWidth));
            advice.AppendLine("│" + CenterText("ЗАКЛЮЧЕНИЕ", Constants.displayWidth - 2) + "│");
            advice.AppendLine(CreateHorizontalLine('├', '─', '┤', Constants.displayWidth));

            if (strengths.Any())
            {
                advice.AppendLine("│ Сильные стороны:".PadRight(Constants.displayWidth - 2) + "│");
                foreach (var strength in strengths)
                {
                    advice.AppendLine("│ • " + strength.PadRight(Constants.displayWidth - 6) + "│");
                }
            }

            if (weaknesses.Any())
            {
                advice.AppendLine("│ Области для улучшения:".PadRight(Constants.displayWidth - 2) + "│");
                foreach (var weakness in weaknesses)
                {
                    advice.AppendLine("│ • " + weakness.PadRight(Constants.displayWidth - 6) + "│");
                }
            }

            string overallAssessment;
            if (strengths.Count > weaknesses.Count)
            {
                overallAssessment = "Хороший уровень игры с потенциалом для роста";
            }
            else if (strengths.Count == weaknesses.Count)
            {
                overallAssessment = "Сбалансированный уровень игры, требуется работа над слабыми сторонами";
            }
            else
            {
                overallAssessment = "Требуется значительная работа над улучшением ключевых аспектов игры";
            }

            advice.AppendLine(CreateHorizontalLine('├', '─', '┤', Constants.displayWidth));
            advice.AppendLine("│" + CenterText(overallAssessment, Constants.displayWidth - 2) + "│");

            return advice.ToString();
        }

        private string CombineColumns(params string[] columns)
        {
            var lines = columns.Select(col => col.Split('\n')).ToArray();
            var maxLines = lines.Max(col => col.Length);
            var result = new System.Text.StringBuilder();

            for (int i = 0; i < maxLines; i++)
            {
                for (int j = 0; j < columns.Length; j++)
                {
                    if (i < lines[j].Length)
                    {
                        result.Append(lines[j][i].TrimEnd());
                        result.Append("   ");
                    }
                }
                result.AppendLine();
            }

            return result.ToString();
        }

        private void DetectInterceptions()
        {
            if (originalCoordinates.Count < 2) return;

            foreach (var player in playerStats)
            {
                player.InterceptionStats = new PlayerInterceptionStats();
            }

            var ballCoordinates = originalCoordinates[0];
            Dictionary<int, int> lastInterceptionFrame = new Dictionary<int, int>();
            Random random = new Random(42);

            for (int frame = 10; frame < ballPossessionSequence.Count - 10; frame++)
            {
                int currentPossessor = ballPossessionSequence[frame];
                int previousPossessor = ballPossessionSequence[frame - 1];

                if (currentPossessor == previousPossessor || currentPossessor == -1 || previousPossessor == -1)
                    continue;

                bool wasTeam1 = previousPossessor < 3;
                bool isTeam1 = currentPossessor < 3;

                if (wasTeam1 == isTeam1) continue;

                var ballPos = ballCoordinates[frame];
                var prevBallPos = ballCoordinates[frame - 5];

                double passDistance = Utilities.CalculateHaversineDistance(prevBallPos, ballPos);
                if (passDistance < 3.0) continue;

                var potentialInterceptors = Enumerable.Range(0, originalCoordinates.Count - 1)
                    .Where(p => (p < 3) != wasTeam1)
                    .Select(p => new
                    {
                        PlayerId = p,
                        Position = originalCoordinates[p][frame],
                        Distance = Utilities.CalculateHaversineDistance(ballPos, originalCoordinates[p][frame])
                    })
                    .Where(p => p.Distance <= 5.0)
                    .OrderBy(p => p.Distance)
                    .ToList();

                foreach (var interceptor in potentialInterceptors)
                {
                    if (lastInterceptionFrame.ContainsKey(interceptor.PlayerId) &&
                        frame - lastInterceptionFrame[interceptor.PlayerId] < MIN_FRAMES_BETWEEN_INTERCEPTIONS)
                        continue;

                    bool wasClean = false;
                    if (frame + 1 < ballPossessionSequence.Count && 
                        frame + 2 < ballPossessionSequence.Count)
                    {
                        wasClean = ballPossessionSequence[frame + 1] == interceptor.PlayerId || 
                                 ballPossessionSequence[frame + 2] == interceptor.PlayerId;
                    }

                    double qualityScore = (1.0 - (interceptor.Distance / 5.0)) *
                                        (wasClean ? 1.2 : 0.8) *
                                        (passDistance > 10.0 ? 1.2 : 1.0);
                    qualityScore = Math.Min(1.0, qualityScore);

                    var interception = new Interception
                    {
                        InterceptingPlayerId = interceptor.PlayerId,
                        PassingPlayerId = previousPossessor,
                        IntendedReceiverId = -1,
                        TimeIndex = frame,
                        Position = ballPos,
                        InterceptionDistance = interceptor.Distance,
                        PassDistance = passDistance,
                        InterceptionQuality = qualityScore
                    };

                    playerStats[interceptor.PlayerId].InterceptionStats.Interceptions.Add(interception);
                    lastInterceptionFrame[interceptor.PlayerId] = frame;

                    if (qualityScore > 0.8 || passDistance > 10.0)
                    {
                        string position = interceptor.PlayerId % 3 == 0 ? "Defender" :
                                        interceptor.PlayerId % 3 == 1 ? "Midfielder" : "Forward";

                        Console.WriteLine($"\nSignificant interception by Player {interceptor.PlayerId + 1} ({position}):");
                        Console.WriteLine($"Pass distance: {passDistance:F1}m");
                        Console.WriteLine($"Interception distance: {interceptor.Distance:F1}m");
                        Console.WriteLine($"Quality score: {qualityScore:F2}");
                        Console.WriteLine($"Clean interception: {wasClean}");
                    }
                }
            }

            Console.WriteLine("\nInterception Statistics:");
            foreach (var player in playerStats)
            {
                var stats = player.InterceptionStats;
                if (stats.TotalInterceptions > 0)
                {
                    int playerIndex = playerStats.IndexOf(player);
                    string position = playerIndex % 3 == 0 ? "Defender" :
                                    playerIndex % 3 == 1 ? "Midfielder" : "Forward";

                    Console.WriteLine($"\nPlayer {playerIndex + 1} ({position}):");
                    Console.WriteLine($"Total interceptions: {stats.TotalInterceptions}");
                    Console.WriteLine($"Average distance: {stats.AverageInterceptionDistance:F1}m");
                    Console.WriteLine($"Average quality: {stats.AverageInterceptionQuality:F2}");

                    if (stats.BestInterceptions.Any())
                    {
                        Console.WriteLine("\nBest interceptions:");
                        foreach (var interception in stats.BestInterceptions)
                        {
                            Console.WriteLine($"- Quality: {interception.InterceptionQuality:F2}");
                            Console.WriteLine($"  Distance: {interception.InterceptionDistance:F1}m");
                            Console.WriteLine($"  Pass length: {interception.PassDistance:F1}m");
                        }
                    }
                }
            }
        }
    }

    public class Program
    {
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new GPSTrackForm());
        }
    }
}
