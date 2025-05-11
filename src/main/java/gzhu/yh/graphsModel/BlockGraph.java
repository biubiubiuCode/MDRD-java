package gzhu.yh.graphsModel;

import gzhu.yh.util.Pair;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

@Getter
@Setter
@NoArgsConstructor
public class BlockGraph extends Graph {

    // 割点列表
    private List<Integer> cutVertices;
    // 块列表
    private List<List<Integer>> blocks;

    // 构造函数
    public BlockGraph(Integer v, Integer e, List<List<Integer>> adjMatrix, List<Pair<Integer, Integer>> edges) {
        super(v, e, adjMatrix, edges);
        this.cutVertices = new ArrayList<>();
        this.blocks = new ArrayList<>();
    }

    @Override
    public void setGraphType() {
        this.setGraphType("Block Graph") ;
    }

    // 查找割点和块
    private void findBlocksAndCutVertices(int u, boolean[] visited, int[] disc, int[] low, int[] parent, boolean[] isCutVertex, Stack<Integer> vertexStack) {
        visited[u] = true;
        disc[u] = low[u] = ++time;
        vertexStack.push(u);

        int children = 0;

        for (int v : this.getAdjList().get(u)) {
            if (!visited[v]) {
                children++;
                parent[v] = u;
                findBlocksAndCutVertices(v, visited, disc, low, parent, isCutVertex, vertexStack);
                low[u] = Math.min(low[u], low[v]);

                // 判断割点
                if (parent[u] != -1 && low[v] >= disc[u]) {
                    isCutVertex[u] = true;

                    List<Integer> block = new ArrayList<>();
                    int w;
                    do {
                        w = vertexStack.pop();
                        block.add(w);
                    } while (w != v);  // 以 v 为终止条件
                    block.add(u);
                    blocks.add(block);
                }
            } else if (v != parent[u]) {
                low[u] = Math.min(low[u], disc[v]);
            }
        }

        // 根节点单独判断
        if (parent[u] == -1 && children > 1) {
            isCutVertex[u] = true;
        }
    }

    // 计算所有的割点和块
    public void analyzeBlocksAndCutVertices() {
        int n = this.getV();
        boolean[] visited = new boolean[n];
        int[] disc = new int[n];
        int[] low = new int[n];
        int[] parent = new int[n];
        boolean[] isCutVertex = new boolean[n];
        Stack<Integer> stack = new Stack<>();

        for (int i = 0; i < n; i++) {
            parent[i] = -1;
        }

        this.blocks = new ArrayList<>();
        this.cutVertices = new ArrayList<>();
        time = 0;

        // 对每个未访问的节点执行DFS
        for (int i = 0; i < n; i++) {
            if (!visited[i]) {
                findBlocksAndCutVertices(i, visited, disc, low, parent, isCutVertex, stack);

                // 清理剩余块
                if (!stack.isEmpty()) {
                    List<Integer> block = new ArrayList<>();
                    while (!stack.isEmpty()) {
                        block.add(stack.pop());
                    }
                    blocks.add(block);
                }
            }
        }

        // 提取割点
        for (int i = 0; i < n; i++) {
            if (isCutVertex[i]) {
                cutVertices.add(i);
            }
        }
    }

    // 随机生成 BlockGraph
    public static BlockGraph randomGenBlockGraphByVertexNum(Integer vertexNum) {
        // 随机生成顶点数为 vertexNum 的图
        // 生成的随机图暂时忽略边数等生成逻辑

        List<List<Integer>> adjMatrix = new ArrayList<>();
        List<Pair<Integer, Integer>> edges = new ArrayList<>();

        // 初始化邻接表
        for (int i = 0; i < vertexNum; i++) {
            adjMatrix.add(new ArrayList<>());
        }

        // 添加随机边（示例代码，具体随机逻辑可根据需求修改）
        for (int i = 0; i < vertexNum; i++) {
            for (int j = i + 1; j < vertexNum; j++) {
                if (Math.random() > 0.5) { // 随机生成边，概率50%
                    adjMatrix.get(i).add(j);
                    adjMatrix.get(j).add(i);
                    edges.add(new Pair<>(i, j));
                }
            }
        }

        BlockGraph blockGraph = new BlockGraph(vertexNum, edges.size(), adjMatrix, edges);
        blockGraph.setGraphType();  // 设置图的类型
        return blockGraph;
    }

    // 用于 DFS 遍历时记录时间戳
    private int time = 0;
}
