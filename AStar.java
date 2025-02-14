package com.atguigu.order.config;// 导入必要的Java库
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.*;

/**
 * A*路径规划算法演示程序主窗口
 * 使用Swing实现图形界面
 */
public class AStarDemo extends JFrame {
    // 界面参数配置
    private final int ROWS = 15;         // 网格行数
    private final int COLS = 15;         // 网格列数
    private final int CELL_SIZE = 40;    // 每个单元格像素大小

    private Node[][] grid;               // 存储所有网格节点
    private Node start;                  // 路径起点
    private Node end;                    // 路径终点
    private String currentMode = "start"; // 当前操作模式

    public static void main(String[] args) {
        // 使用事件调度线程启动GUI
        SwingUtilities.invokeLater(() -> new AStarDemo().setVisible(true));
    }

    public AStarDemo() {
        // 窗口初始化设置
        setTitle("A*路径规划演示程序");
        setSize(COLS * CELL_SIZE + 20, ROWS * CELL_SIZE + 100); // 计算合适窗口大小
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLayout(new BorderLayout());  // 使用边界布局

        initializeGrid();  // 初始化网格数据

        // 创建控制面板
        JPanel controlPanel = new JPanel();
        // 创建功能按钮
        JButton btnStart = new JButton("设置起点");
        JButton btnEnd = new JButton("设置终点");
        JButton btnBarrier = new JButton("设置障碍");
        JButton btnRun = new JButton("开始寻路");
        JButton btnClear = new JButton("清除所有");

        // 设置按钮事件监听器
        btnStart.addActionListener(e -> currentMode = "start");
        btnEnd.addActionListener(e -> currentMode = "end");
        btnBarrier.addActionListener(e -> currentMode = "barrier");
        btnRun.addActionListener(e -> runAStar());  // 触发路径查找
        btnClear.addActionListener(e -> clearAll()); // 重置所有数据

        // 将按钮添加到控制面板
        controlPanel.add(btnStart);
        controlPanel.add(btnEnd);
        controlPanel.add(btnBarrier);
        controlPanel.add(btnRun);
        controlPanel.add(btnClear);

        // 将组件添加到窗口
        add(controlPanel, BorderLayout.NORTH);
        add(new GridPanel(), BorderLayout.CENTER); // 添加网格绘制面板
    }

    /**
     * 初始化网格数据
     * 创建ROWS×COLS的节点矩阵
     */
    private void initializeGrid() {
        grid = new Node[ROWS][COLS];
        for (int i = 0; i < ROWS; i++) {
            for (int j = 0; j < COLS; j++) {
                grid[i][j] = new Node(i, j);
            }
        }
    }

    /**
     * 重置所有节点状态
     * 清空起点/终点/障碍物设置
     */
    private void clearAll() {
        initializeGrid();  // 重新初始化网格
        start = null;
        end = null;
        repaint();  // 触发界面重绘
    }

    /**
     * A*算法核心实现
     * 使用优先队列优化搜索过程
     */
    private void runAStar() {
        // 检查起点终点是否设置
        if (start == null || end == null) {
            JOptionPane.showMessageDialog(this, "请先设置起点和终点");
            return;
        }

        // 算法数据结构初始化
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingInt(n -> n.f)); // 开放列表（按f值排序）
        Set<Node> closedSet = new HashSet<>(); // 关闭列表
        resetNodes(); // 重置节点状态

        // 初始化起点
        openSet.add(start);
        start.g = 0;  // 起点到自身的代价为0
        start.f = start.h = heuristic(start, end); // 计算启发值

        // 主循环过程
        while (!openSet.isEmpty()) {
            Node current = openSet.poll();  // 获取当前最优节点

            // 找到终点时重建路径
            if (current == end) {
                reconstructPath(current);
                return;
            }

            closedSet.add(current);  // 将当前节点加入关闭列表

            // 遍历四个方向的邻居节点
            for (Node neighbor : getNeighbors(current)) {
                // 跳过不可通过或已处理的节点
                if (!neighbor.walkable || closedSet.contains(neighbor)) continue;

                // 计算新的移动代价
                int tentativeG = current.g + 1; // 假设每步代价为1

                // 发现更优路径时更新节点
                if (tentativeG < neighbor.g) {
                    neighbor.parent = current;  // 记录父节点用于路径回溯
                    neighbor.g = tentativeG;    // 更新实际代价
                    neighbor.h = heuristic(neighbor, end); // 计算启发值
                    neighbor.f = neighbor.g + neighbor.h; // 计算总评估值

                    // 如果邻居不在开放列表则添加
                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }
        }
        // 开放列表为空表示无解
        JOptionPane.showMessageDialog(this, "找不到可行路径！");
    }

    /**
     * 曼哈顿距离启发函数
     * @param a 当前节点
     * @param b 目标节点
     * @return 两节点间的估算距离
     */
    private int heuristic(Node a, Node b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    /**
     * 路径回溯方法
     * 从终点沿父节点链回溯，标记路径节点
     * @param current 终点节点
     */
    private void reconstructPath(Node current) {
        while (current != null) {
            if (current != start && current != end) {
                current.isPath = true;  // 标记为路径节点
            }
            current = current.parent;  // 向上回溯
        }
        repaint();  // 触发界面更新
    }

    /**
     * 获取相邻节点
     * @param node 当前节点
     * @return 有效邻居节点列表（上下左右四个方向）
     */
    private ArrayList<Node> getNeighbors(Node node) {
        ArrayList<Node> neighbors = new ArrayList<>();
        int[][] directions = {{-1,0}, {1,0}, {0,-1}, {0,1}}; // 四方向移动

        for (int[] d : directions) {
            int nx = node.x + d[0];  // 计算新坐标
            int ny = node.y + d[1];

            // 检查边界有效性
            if (nx >= 0 && nx < ROWS && ny >= 0 && ny < COLS) {
                neighbors.add(grid[nx][ny]);
            }
        }
        return neighbors;
    }

    /**
     * 重置节点算法参数
     * 每次寻路前都需要执行
     */
    private void resetNodes() {
        for (Node[] row : grid) {
            for (Node node : row) {
                node.g = Integer.MAX_VALUE; // 重置实际代价
                node.f = node.h = 0;         // 重置评估值
                node.parent = null;         // 清除父节点
                node.isPath = false;         // 清除路径标记
            }
        }
    }

    /**
     * 网格绘制面板
     * 处理界面绘制和鼠标交互
     */
    class GridPanel extends JPanel {
        /**
         * 自定义绘制方法
         * @param g 图形上下文对象
         */
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);

            // 遍历所有节点进行绘制
            for (int i = 0; i < ROWS; i++) {
                for (int j = 0; j < COLS; j++) {
                    Node node = grid[i][j];
                    Color color = Color.WHITE;  // 默认白色

                    // 根据节点状态设置颜色
                    if (node == start) {
                        color = Color.GREEN;     // 起点：绿色
                    } else if (node == end) {
                        color = Color.RED;       // 终点：红色
                    } else if (!node.walkable) {
                        color = Color.BLACK;     // 障碍：黑色
                    } else if (node.isPath) {
                        color = Color.BLUE;      // 路径：蓝色
                    }

                    // 绘制单元格
                    g.setColor(color);
                    g.fillRect(j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE);
                    g.setColor(Color.GRAY);  // 网格线颜色
                    g.drawRect(j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE);
                }
            }
        }

        /**
         * 网格面板构造函数
         * 添加鼠标点击监听
         */
        public GridPanel() {
            addMouseListener(new MouseAdapter() {
                @Override
                public void mouseClicked(MouseEvent e) {
                    // 计算点击位置对应的网格坐标
                    int col = e.getX() / CELL_SIZE;
                    int row = e.getY() / CELL_SIZE;
                    Node node = grid[row][col];

                    // 根据当前模式处理点击
                    switch (currentMode) {
                        case "start":
                            if (node.walkable) {
                                start = node;  // 设置起点
                                repaint();
                            }
                            break;
                        case "end":
                            if (node.walkable) {
                                end = node;    // 设置终点
                                repaint();
                            }
                            break;
                        case "barrier":
                            node.walkable = !node.walkable; // 切换障碍状态
                            repaint();
                            break;
                    }
                }
            });
        }
    }

    /**
     * 节点类
     * 表示网格中的每个单元格
     */
    class Node {
        int x, y;           // 网格坐标
        int g = Integer.MAX_VALUE; // 到起点的实际代价
        int h, f;           // 启发值、总评估值(f=g+h)
        boolean walkable = true;  // 是否可通过
        boolean isPath = false;   // 是否在最终路径上
        Node parent;        // 路径回溯父节点

        public Node(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }
}
