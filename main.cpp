#include "viewer/gl_mesh/gl_include.hpp"
#include "viewer/gl_mesh/container.h"
#include <iostream>
#include "MarchingCubes.h"

using namespace std;

int main(int argc, char **argv){
    QApplication a(argc,argv);
    MainWindow *window = new MainWindow();
    window->show();
    a.exec();

//    std::vector<float> leaf(8);
//    leaf[0]=-1.0f;
//    leaf[1]=1.0f;
//    leaf[2]=1.0f;
//    leaf[3]=1.0f;
//    leaf[4]=1.0f;
//    leaf[5]=1.0f;
//    leaf[6]=1.0f;
//    leaf[7]=1.0f;
//
//    std::vector<MeshPoint> points(8);
//
//    points[0].meshPoint[0]=0.0f;
//    points[0].meshPoint[1]=0.0f;
//    points[0].meshPoint[2]=0.0f;
//    points[0].meshPoint[3]=1.0f;
//    points[0].meshPoint[4]=0.0f;
//    points[0].meshPoint[5]=0.0f;
//
//    points[1].meshPoint[0]=0.0f;
//    points[1].meshPoint[1]=6.0f;
//    points[1].meshPoint[2]=0.0f;
//    points[1].meshPoint[3]=0.0f;
//    points[1].meshPoint[4]=0.0f;
//    points[1].meshPoint[5]=1.0f;
//
//    points[2].meshPoint[0]=6.0f;
//    points[2].meshPoint[1]=6.0f;
//    points[2].meshPoint[2]=0.0f;
//    points[2].meshPoint[3]=1.0f;
//    points[2].meshPoint[4]=0.0f;
//    points[2].meshPoint[5]=0.0f;
//
//    points[3].meshPoint[0]=6.0f;
//    points[3].meshPoint[1]=0.0f;
//    points[3].meshPoint[2]=0.0f;
//    points[3].meshPoint[3]=0.0f;
//    points[3].meshPoint[4]=1.0f;
//    points[3].meshPoint[5]=0.0f;
//
//    points[4].meshPoint[0]=0.0f;
//    points[4].meshPoint[1]=0.0f;
//    points[4].meshPoint[2]=6.0f;
//    points[4].meshPoint[3]=0.0f;
//    points[4].meshPoint[4]=1.0f;
//    points[4].meshPoint[5]=0.0f;
//
//    points[5].meshPoint[0]=0.0f;
//    points[5].meshPoint[1]=6.0f;
//    points[5].meshPoint[2]=6.0f;
//    points[5].meshPoint[3]=1.0f;
//    points[5].meshPoint[4]=0.0f;
//    points[5].meshPoint[5]=0.0f;
//
//    points[6].meshPoint[0]=6.0f;
//    points[6].meshPoint[1]=6.0f;
//    points[6].meshPoint[2]=6.0f;
//    points[6].meshPoint[3]=1.0f;
//    points[6].meshPoint[4]=0.0f;
//    points[6].meshPoint[5]=0.0f;
//
//    points[7].meshPoint[0]=6.0f;
//    points[7].meshPoint[1]=0.0f;
//    points[7].meshPoint[2]=6.0f;
//    points[7].meshPoint[3]=0.0f;
//    points[7].meshPoint[4]=0.0f;
//    points[7].meshPoint[5]=1.0f;
//
//
//    std::vector<faces> ret;
//    ret.reserve(20);
//    MarchingCubes* m=new MarchingCubes();
//    m->getMeshSurface(leaf,points,ret);
//
//    for(int i=0;i<ret.size();i++){
//
//        std::cout<<ret[i].face[0]<<" "<<ret[i].face[1]<<" "<<ret[i].face[2]<<" "
//        <<ret[i].face[3]<<" "<<ret[i].face[4]<<" "<<ret[i].face[5]<<std::endl;
//        std::cout<<ret[i].face[6]<<" "<<ret[i].face[7]<<" "<<ret[i].face[8]<<" "
//                 <<ret[i].face[9]<<" "<<ret[i].face[10]<<" "<<ret[i].face[11]<<std::endl;
//        std::cout<<ret[i].face[12]<<" "<<ret[i].face[13]<<" "<<ret[i].face[14]<<" "
//                 <<ret[i].face[15]<<" "<<ret[i].face[16]<<" "<<ret[i].face[17]<<std::endl;
//
//    }
    return 0;
}