#include <cstdlib>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <map>
#include <queue>
#include<tuple>
#include <stdlib.h>
#include <cmath>
#include <chrono>

using namespace std;
using namespace std::chrono; 

typedef tuple<std::string, int> bfs_path_cost;
typedef std::pair<int, int> xy_coordinates;

typedef std::map<xy_coordinates, bfs_path_cost> bfs_visited_map;

// the XY coordinates, path to ucs, the cost of the path of ucs.
typedef tuple<xy_coordinates, std::string, int> ucs_path_cost;

//the XY coordinates, path to A*, the cost of the path including heuristic and path length of A* without heuristic.
typedef tuple<xy_coordinates, std::string, int, int> astar_path_cost;

uint32_t absDiff(int x, int y)
{
    if ( x < y )    
        return y - x;
    else             
        return x - y;
}

class CompareDist
{
public:
    bool operator()(ucs_path_cost n1,ucs_path_cost n2) {
        return get<2>(n1) > get<2>(n2);
    }
    bool operator()(astar_path_cost n1,astar_path_cost n2) {
        return get<2>(n1) > get<2>(n2);
    }
};


class search_t {

private:

string algo_name;
uint32_t W;
uint32_t H;
uint32_t landing_Y;
uint32_t landing_X;
uint32_t elevation_dis;
uint32_t N_tgts;
//vector<vector <int> > array;
int **array;
uint32_t **coordinates;

int eucledian_dis(int x1, int y1, int x2, int y2)
{

    return sqrt(pow(x2-x1,2)+pow(y2-y1,2));

}

public:

~search_t()
{
    for(int i = 0 ; i < H; i++)
        delete array[i];

    delete array;

    for(int i = 0 ; i < N_tgts; i++)
        delete coordinates[i];

    delete coordinates;
}

void readParams()
{
/*
    std::ifstream file("input.txt");

    if(file.is_open())
    {
    
        std::string line;

        while(getline(file,line)){
        
            printf("%s\n", line.c_str());

        }
    }

    file.close();
*/

    //FILE *fp;

    //fp = freopen("input.txt","r",stdin);

    ifstream file( "input.txt", ios::in );

    file >> algo_name;
    file >> W;
    file >> H;
    file >> landing_Y;
    file >> landing_X;
    file >> elevation_dis;
    file >> N_tgts;
   

//    cout << "The algo name is " +algo_name;

    coordinates = new uint32_t*[N_tgts];

    for( int j = 0 ; j < N_tgts ; j++)
    {
        coordinates[j] = new uint32_t[2];
    }


    for( int i = 0 ; i < N_tgts ; i++ )
    {
        for( int j = 0 ; j < 2 ; j++)
        {
            file >> coordinates[i][j];
        }
    }

//    cout << "The N-tgts is" << N_tgts;
    array = new int*[H];

    for( int i = 0; i < H ; i++ )
    {
        array[i] = new int[W];
    }

    for( int i = 0; i < H; i++ )
    {
        for( int j = 0; j < W ;j++ )
        {
            file >> array[i][j];
        }
    }

#if 0
    cout<< "THe matrix is:\n" << endl;

    for( int i = 0; i < H; i++ )
    {
        for( int j = 0; j < W ;j++ )
        {
            cout << array[i][j] << "\t";
        }
        cout <<endl;
    }
#endif



//    cout << "The N-tgts is" << N_tgts;
    file.close();
}

void show()
{

    cout << "The algo name is " +algo_name;
    cout << "The N-tgts is" << N_tgts;
    for( int i = 0; i < H; i++ )
    {
        for( int j = 0; j < W ; j++ )
        {
            cout << array[i][j] << "\t";
        }

        cout << endl;
    }

}

void display(string str)
{

    //FILE *fp;

    //freopen("output.txt","w",stdout);

    ofstream fout;

    fout.open("output.txt");

    fout << str << endl;

    fout.close();

}

void callSearch()
{
    //cout<< __func__ << N_tgts;
    for( int i = 0 ; i < N_tgts ; i++ )
    {
        xy_coordinates dest = make_pair(coordinates[i][1], coordinates[i][0]);
        
        if( algo_name == "BFS" )
        {
            BFS_inner(dest);
        }
        else if( algo_name == "UCS")
        {
            UCS_inner(dest);
        }
        else if( algo_name == "A*")
        {
            Astar_inner(dest);
        }
        //cout << "Exiting" << __func__ ;
    }
    //cout << "Exiting" << __func__ ;

}

void BFS_inner(xy_coordinates dest)
{
    //cout << __func__ ;
    std::queue<xy_coordinates> q;

    bfs_visited_map visited_map;

    xy_coordinates src = make_pair(landing_X,landing_Y);
    xy_coordinates visited_node_name;
    xy_coordinates next_neighbor;

    bfs_visited_map::iterator itr;

    q.push(src);

    bfs_path_cost bfs_path_var(to_string(landing_Y)+","+to_string(landing_X), 0);

    visited_map.insert(pair<xy_coordinates, bfs_path_cost>(src,bfs_path_var));

    while(!q.empty())
    {

        //cout<< q.size() << endl;
        xy_coordinates node = q.front();
        q.pop();

        itr = visited_map.find(node);

        bfs_path_cost value;
         
         // get the node's values of path and cost
        if(itr != visited_map.end())
        {
            value = itr->second;
        }

        //cout<< "!!!!The value of path is:"+get<0>(value)<< endl;
        //cout<< "Elevation_distance" << elevation_dis <<endl;
        if( node == dest)
        {    
            //cout<< "The value of path is:"+get<0>(value)<< endl;
            //cout<<"Exiting\n"<<endl;
            display(get<0>(value));
            return;
        }

        int row = get<0>(node);
        int col = get<1>(node);

        uint32_t src_elevation;
        
        src_elevation = array[row][col];

        // moving up 
        if (row - 1 >= 0) {
            
            next_neighbor = make_pair(row-1, col);
            if(visited_map.find(next_neighbor) == visited_map.end() &&
                            absDiff(src_elevation, array[row-1][col]) <= elevation_dis )
            {
                // cout<< row-1 << col <<"Done"<< endl ;
                //cout<< "Inserting1" << next_neighbor.first << "," << next_neighbor.second <<"Done" <<endl;
                q.push(next_neighbor);

                bfs_path_var = make_tuple(get<0>(value)+" "+to_string(col)+","+to_string(row-1), int(get<1>(value))+1);

                visited_map.insert(pair<xy_coordinates, bfs_path_cost>(next_neighbor, bfs_path_var));
            }

            // up left diagonal
            next_neighbor = make_pair(row-1, col - 1);
            if(col - 1 >= 0 && visited_map.find(next_neighbor) == visited_map.end() &&
                            absDiff(src_elevation, array[row-1][col-1]) <= elevation_dis )
            {
                q.push(next_neighbor);
                //cout<< "Inserting2" << next_neighbor.first << "," << next_neighbor.second <<"Done" <<endl;

                bfs_path_var = make_tuple(get<0>(value)+" "+to_string(col-1)+","+to_string(row-1), get<1>(value)+1);

                visited_map.insert(pair<xy_coordinates, bfs_path_cost>(next_neighbor,bfs_path_var));
            }
            
            // up right diagonal
            next_neighbor = make_pair(row-1, col + 1);
            if(col + 1 <= W && visited_map.find(next_neighbor) == visited_map.end() &&
                            absDiff(src_elevation, array[row-1][col+1]) <= elevation_dis )
            {
                q.push(next_neighbor);

                //cout<< "Inserting3" << next_neighbor.first << "," << next_neighbor.second <<"Done" <<endl;
                bfs_path_var = make_tuple(get<0>(value)+" "+to_string(col+1)+","+to_string(row-1), get<1>(value)+1);

                visited_map.insert(pair<xy_coordinates, bfs_path_cost>(next_neighbor,bfs_path_var));
            }
        }
  
        // moving down 
        if (row + 1 < H) { 

            next_neighbor = make_pair(row+1, col);
            if(visited_map.find(next_neighbor) == visited_map.end() &&
                            absDiff(src_elevation, array[row+1][col]) <= elevation_dis )
            {
                q.push(next_neighbor);

                //cout<< "Inserting4" << next_neighbor.first << "," << next_neighbor.second <<"Done" <<endl;
                bfs_path_var = make_tuple(get<0>(value)+" "+to_string(col)+","+to_string(row+1), get<1>(value)+1);

                visited_map.insert(pair<xy_coordinates, bfs_path_cost>(next_neighbor,bfs_path_var));
            }

            // down left diagonal
            next_neighbor = make_pair(row+1, col - 1);
            if(col - 1 >= 0 && visited_map.find(next_neighbor) == visited_map.end() &&
                            absDiff(src_elevation, array[row+1][col-1]) <= elevation_dis )
            {
                q.push(next_neighbor);

                //cout<< "Inserting5" << next_neighbor.first << "," << next_neighbor.second <<"Done" <<endl;
                bfs_path_var = make_tuple(get<0>(value)+" "+to_string(col-1)+","+to_string(row+1), get<1>(value)+1);

                visited_map.insert(pair<xy_coordinates, bfs_path_cost>(next_neighbor,bfs_path_var));
            }

            // down right diagonal
            next_neighbor = make_pair(row+1, col+1);
            if(col + 1 <= W && visited_map.find(next_neighbor) == visited_map.end() &&
                            absDiff(src_elevation, array[row+1][col+1]) <= elevation_dis )
            {
                q.push(next_neighbor);

                //cout<< "Inserting6" << next_neighbor.first << "," << next_neighbor.second <<"Done" <<endl;
                bfs_path_var = make_tuple(get<0>(value)+" "+to_string(col+1)+","+to_string(row+1), get<1>(value)+1);

                visited_map.insert(pair<xy_coordinates, bfs_path_cost>(next_neighbor,bfs_path_var));
            }
        } 
  
        next_neighbor = make_pair(row, col-1);
        // moving left 
        if (col - 1 >= 0 && visited_map.find(next_neighbor) == visited_map.end()
                        && absDiff(src_elevation, array[row][col-1]) <= elevation_dis )
        { 
            //cout<< "Inserting7" << next_neighbor.first << "," << next_neighbor.second <<"Done" <<endl;
            q.push(next_neighbor);

            bfs_path_var = make_tuple(get<0>(value)+" "+to_string(col-1)+","+to_string(row), get<1>(value)+1);

            visited_map.insert(pair<xy_coordinates, bfs_path_cost>(next_neighbor,bfs_path_var));
        } 
  
        next_neighbor = make_pair(row, col+1);
         // moving right 
        if (col + 1 <= W && visited_map.find(next_neighbor) == visited_map.end() &&
                        absDiff(src_elevation, array[row][col+1]) <= elevation_dis ) 
        { 

            //cout<< "Inserting8" << next_neighbor.first << "," << next_neighbor.second <<"Done" <<endl;
            q.push(next_neighbor);

            bfs_path_var = make_tuple(get<0>(value)+" "+to_string(col+1)+","+to_string(row), get<1>(value)+1);

            visited_map.insert(pair<xy_coordinates, bfs_path_cost>(next_neighbor,bfs_path_var));
        } 
        
    }

    //cout << "FAIL" << endl;
    display("FAIL");

}

/*
void BFS(uint32_t dest_row, uint32_t dest_col)
{
    std::queue<xy_coordinates> q;

    uint32_t src_row = landing_X;
    uint32_t src_col = landing_Y;

    int **visited;

    visited = new int*[H];
    for(int i = 0 ; i < H; i++)
    {
        visited[i] = new int[W]();
    }

    xy_coordinates src_xy = make_pair(src_row, src_col);

    q.push(src_xy);

    while(!q.empty())
    {
        xy_coordinates q_item = q.top();

        int q_item_row = q_item.first;

        int q_item_col = q_item.second;

        if(q_item_row == dest_row && q_item_col == dest_col)
        {
        
        }
    
    
    }




}
*/

void UCS_inner(xy_coordinates dest)
{

    priority_queue<ucs_path_cost,vector<ucs_path_cost>,CompareDist> pq;

    int **visited;

    visited = new int*[H];

    for(int i = 0 ; i < H ; i++)
    {
        visited[i] = new int[W]();
    }

    //cout<<"W"<< W << "H"<< H;

    //bfs_visited_map visited_map;
    
    xy_coordinates XY = make_pair(landing_X, landing_Y);

    ucs_path_cost q_item = make_tuple(XY, to_string(landing_Y)+","+to_string(landing_X), 0);

    pq.push(q_item);
    //bfs_path_cost bfs_path_var(to_string(landing_X)+","+to_string(landing_Y), 0);

    while(!pq.empty())
    {
        q_item = pq.top();
        pq.pop();   

        XY = get<0>(q_item);
        std::string path = get<1>(q_item);
        int cost  = get<2>(q_item);

        int row = XY.first;
        int col = XY.second;

        //cout<< "X:"<< row << "," <<"Y:"<< col <<endl;

        //cout<< path <<endl;
        xy_coordinates curr = make_pair(row, col);

        if(visited[row][col])
            continue;

        visited[row][col] = 1;

        //visited_map.insert(pair<xy_coordinates, bfs_path_cost>(curr,bfs_path_var));
        
        if ( curr == dest)
        {
            
            //cout << "The path is:" << path << endl;
            //cout<< "The cost is:" << cost << endl;
            display(path);
            return;
        }

        uint32_t src_elevation;
        
        src_elevation = array[row][col];

        // moving up
        if(row - 1 >= 0)
        {
            if(!visited[row-1][col] && 
                (absDiff(src_elevation, array[row-1][col]) <= elevation_dis))
            {
                int new_cost = cost + 10;

                XY = make_pair(row-1,col);

                q_item = make_tuple(XY, path+" "+to_string(col)+","+to_string(row -1), new_cost);
                pq.push(q_item);
          }

            // up left diagonal
            if(col - 1 >= 0 && !visited[row-1][col-1] &&
                    (absDiff(src_elevation, array[row-1][col-1]) <= elevation_dis))
            {
                int new_cost = cost + 14;

                XY = make_pair(row-1,col-1);

                q_item = make_tuple(XY, path+" "+to_string(col-1)+","+to_string(row-1), new_cost);
                pq.push(q_item);
            }

            // up right diagonal
            if(col + 1 < W && !visited[row-1][col+1] &&
                    (absDiff(src_elevation, array[row-1][col+1]) <= elevation_dis))
            {
                int new_cost = cost + 14;

                XY = make_pair(row-1,col+1);

                q_item = make_tuple(XY, path+" "+to_string(col+1)+","+to_string(row-1), new_cost);
                pq.push(q_item);
            }

        }
       
        // moving down
        if(row + 1 < H)
        {
             if(!visited[row+1][col] && 
                (absDiff(src_elevation, array[row+1][col]) <= elevation_dis))
            {
                int new_cost = cost + 10;

                XY = make_pair(row+1,col);

                q_item = make_tuple(XY, path+" "+to_string(col)+","+to_string(row+1), new_cost);
                pq.push(q_item);
          }

            // lower left diagonal
            if(col - 1 >= 0 && !visited[row+1][col-1] &&
                    (absDiff(src_elevation, array[row+1][col-1]) <= elevation_dis))
            {
                int new_cost = cost + 14;

                XY = make_pair(row+1,col-1);

                q_item = make_tuple(XY, path+" "+to_string(col-1)+","+to_string(row+1), new_cost);
                pq.push(q_item);
            }

            // lower right diagonal
            if(col + 1 < W && !visited[row+1][col+1] &&
                    (absDiff(src_elevation, array[row+1][col+1]) <= elevation_dis))
            {
                int new_cost = cost + 14;

                XY = make_pair(row+1,col+1);

                q_item = make_tuple(XY, path+" "+to_string(col+1)+","+to_string(row+1), new_cost);
                pq.push(q_item);
            }
        
        }

        if(col - 1 >=0 && !visited[row][col-1]
                        &&  (absDiff(src_elevation, array[row][col-1]) <= elevation_dis))
        {
                int new_cost = cost + 10;

                XY = make_pair(row,col-1);

                q_item = make_tuple(XY, path+" "+to_string(col-1)+","+to_string(row), new_cost);
                pq.push(q_item);
        }

        if(col + 1 < W && !visited[row][col+1]
                        &&  (absDiff(src_elevation, array[row][col+1]) <= elevation_dis))
        {
                int new_cost = cost + 10;

                XY = make_pair(row,col+1);

                q_item = make_tuple(XY, path+" "+to_string(col+1)+","+to_string(row), new_cost);
                pq.push(q_item);
        }

    }
    display("FAIL");
    //cout << "FAIL" << endl;
}

void Astar_inner(xy_coordinates dest)
{

    priority_queue<astar_path_cost,vector<astar_path_cost>,CompareDist> pq;

    int **visited;

    visited = new int*[H];

    for(int i = 0 ; i < H ; i++)
    {
        visited[i] = new int[W];
    }

    for(int i = 0 ; i < H ; i++)
    {
        for(int j = 0; j < W; j++)
        {
            visited[i][j] = 0;
        }
    
    }

    //bfs_visited_map visited_map;
    xy_coordinates XY = make_pair(landing_X, landing_Y);

    astar_path_cost q_item = make_tuple(XY, to_string(landing_Y)+","+to_string(landing_X), eucledian_dis(landing_X, landing_Y, dest.first, dest.second), 0);

    pq.push(q_item);
    //bfs_path_cost bfs_path_var(to_string(landing_X)+","+to_string(landing_Y), 0);

    while(!pq.empty())
    {
        q_item = pq.top();
        pq.pop();   

        XY = get<0>(q_item);
        std::string path = get<1>(q_item);
       // int heuristic_cost  = get<2>(q_item);
        int cost = get<3>(q_item);

        int row = XY.first;
        int col = XY.second;

        xy_coordinates curr = make_pair(row, col);

        if(visited[row][col])
        {
            //cout << "visitedX:" << row << "Y:"<< col << endl;
            continue;
        }
        visited[row][col] = 1;

        //cout << "X:" << row << "Y:"<< col << endl;
        //cout << "The path is:" << path << endl;
        //visited_map.insert(pair<xy_coordinates, bfs_path_cost>(curr,bfs_path_var));
        
        if ( curr == dest)
        {
            
            //cout << "The path is:" << path << endl;
            //cout<< "The cost is:" << cost << endl;
            display(path); 
            return;
        }

        uint32_t src_elevation;
        
        src_elevation = array[row][col];

        uint32_t diff = 0;

        // moving up
        if(row - 1 >= 0)
        {
            if(!visited[row-1][col] && 
                ((diff = absDiff(src_elevation, array[row-1][col])) <= elevation_dis))
            {
                int new_cost = cost + 10 + diff;
                int costHeuristic = new_cost + eucledian_dis(row-1,col, dest.first, dest.second);

                XY = make_pair(row-1,col);
#if 0
                cout << "X:" <<XY.first << "Y:"<<XY.second << endl;
                cout << "new_cost:"<<new_cost << endl;
                cout << "costHeuristic:"<<costHeuristic << endl;

#endif
                q_item = make_tuple(XY, path+" "+to_string(col)+","+to_string(row -1), costHeuristic,new_cost);
                pq.push(q_item);
          }

            // up left diagonal
            if(col - 1 >= 0 && !visited[row-1][col-1] &&
                    ((diff = absDiff(src_elevation, array[row-1][col-1])) <= elevation_dis))
            {
                int new_cost = cost + 14 + diff;
                int costHeuristic = new_cost + eucledian_dis(row-1,col-1, dest.first, dest.second);

                XY = make_pair(row-1,col-1);

#if 0
                cout << "X:" <<XY.first << "Y:"<<XY.second << endl;
                cout << "new_cost:"<<new_cost << endl;
                cout << "costHeuristic:"<<costHeuristic << endl;
                
#endif
                q_item = make_tuple(XY, path+" "+to_string(col-1)+","+to_string(row-1), costHeuristic,new_cost);
                pq.push(q_item);
            }

            // up right diagonal
            if(col + 1 < W && !visited[row-1][col+1] &&
                    ((diff = absDiff(src_elevation, array[row-1][col+1])) <= elevation_dis))
            {
                int new_cost = cost + 14 + diff;
                int costHeuristic = new_cost + eucledian_dis(row-1,col+1, dest.first, dest.second);

                XY = make_pair(row-1,col+1);

#if 0
                cout << "X:" <<XY.first << "Y:"<<XY.second << endl;
                cout << "new_cost:"<<new_cost << endl;
                cout << "costHeuristic:"<<costHeuristic << endl;
                
#endif
                q_item = make_tuple(XY, path+" "+to_string(col+1)+","+to_string(row-1), costHeuristic,new_cost);
                pq.push(q_item);
            }

        }
       
        // moving down
        if(row + 1 < H)
        {
             if(!visited[row+1][col] && 
                ((diff = absDiff(src_elevation, array[row+1][col])) <= elevation_dis))
            {
                int new_cost = cost + 10 + diff;
                int costHeuristic = new_cost + eucledian_dis(row+1,col, dest.first, dest.second);

                XY = make_pair(row+1,col);
#if 0
                cout << "X:" <<XY.first << "Y:"<<XY.second << endl;
                cout << "new_cost:"<<new_cost << endl;
                cout << "costHeuristic:"<<costHeuristic << endl;

#endif

                q_item = make_tuple(XY, path+" "+to_string(col)+","+to_string(row+1), costHeuristic,new_cost);
                pq.push(q_item);
          }

            // lower left diagonal
            if(col - 1 >= 0 && !visited[row+1][col-1] &&
                    ((diff = absDiff(src_elevation, array[row+1][col-1])) <= elevation_dis))
            {
                int new_cost = cost + 14 + diff;
                int costHeuristic = new_cost + eucledian_dis(row+1,col-1, dest.first, dest.second);

                XY = make_pair(row+1,col-1);
#if 0
                cout << "X:" <<XY.first << "Y:"<<XY.second << endl;
                cout << "new_cost:"<<new_cost << endl;
                cout << "costHeuristic:"<<costHeuristic << endl;

#endif

                q_item = make_tuple(XY, path+" "+to_string(col-1)+","+to_string(row+1), costHeuristic,new_cost);
                pq.push(q_item);
            }

            // lower right diagonal
            if(col + 1 < W && !visited[row+1][col+1] &&
                    ((diff = absDiff(src_elevation, array[row+1][col+1])) <= elevation_dis))
            {
                int new_cost = cost + 14 + diff;

                int costHeuristic = new_cost + eucledian_dis(row+1,col+1, dest.first, dest.second);

                XY = make_pair(row+1,col+1);
#if 0
                cout << "X:" <<XY.first << "Y:"<<XY.second << endl;
                cout << "new_cost:"<<new_cost << endl;
                cout << "costHeuristic:"<<costHeuristic << endl;

#endif
                q_item = make_tuple(XY, path+" "+to_string(col+1)+","+to_string(row+1), costHeuristic,new_cost);
                pq.push(q_item);
            }
        
        }

        if(col - 1 >=0 && !visited[row][col-1]
                        &&  ((diff = absDiff(src_elevation, array[row][col-1])) <= elevation_dis))
        {
                int new_cost = cost + 10 + diff;
                int costHeuristic = new_cost + eucledian_dis(row,col-1, dest.first, dest.second);

                XY = make_pair(row,col-1);
#if 0
                cout << "X:" <<XY.first << "Y:"<<XY.second << endl;
                cout << "new_cost:"<<new_cost << endl;
                cout << "costHeuristic:"<<costHeuristic << endl;

#endif
                q_item = make_tuple(XY, path+" "+to_string(col-1)+","+to_string(row), costHeuristic,new_cost);
                pq.push(q_item);
        }

        if(col + 1 < W && !visited[row][col+1]
                        &&  ((diff = absDiff(src_elevation, array[row][col+1])) < elevation_dis))
        {
                int new_cost = cost + 10 + diff;
                int costHeuristic = new_cost + eucledian_dis(row,col+1, dest.first, dest.second);

                XY = make_pair(row,col+1);
#if 0
                cout << "X:" <<XY.first << "Y:"<<XY.second << endl;
                cout << "new_cost:"<<new_cost << endl;
                cout << "costHeuristic:"<<costHeuristic << endl;
#endif
                q_item = make_tuple(XY, path+" "+to_string(col+1)+","+to_string(row), costHeuristic, new_cost);
                pq.push(q_item);
        }

    }
    //cout << "FAIL" << endl;
    display("FAIL");

}

};

int main()
{

    search_t sh;

    auto start = high_resolution_clock::now();
    sh.readParams();

    sh.callSearch();

    auto stop = high_resolution_clock::now();
    //cout << "\nExiting main";
    sh.show();

    auto duration = duration_cast<minutes>(stop - start); 
    cout << "Duration:" << duration.count() << endl;

    return 0;

}



