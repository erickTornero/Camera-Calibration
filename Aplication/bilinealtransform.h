# include <iostream>
# include <vector>
# include <string.h>
# include <chrono>
void printm(float ** m, int nrows, int ncols){
    for(int i = 0; i < nrows; i++){
        for(int j = 0; j < ncols; j++){
            std::cout<<m[i][j]<<"\t";
        }
        std::cout<<std::endl;
    }
}
void printvector(float *m, int sz){
    for(int i = 0; i < sz; i++)
        std::cout<<m[i]<<std::endl;
}

float ** MultiplyMatrixes(float ** matrixA, float ** matrixB, int R1, int C, int C2){
    float ** ans = new float * [R1];
    //Initialize the matrix answer
    for(int m = 0; m < R1; m++)
        ans[m] = new float[C2];

    for(int i = 0; i < R1; i++){
        for(int j = 0; j < C2; j++){
            
            // For each element of matrix:
            float acumm = 0;
            for(int k = 0; k < C; k++){
                acumm += matrixA[i][k]*matrixB[k][j];
            }
            ans[i][j] = acumm;
        }
    }
    return ans;
}
int DotProduct(float * vec1, float * vec2, int sz){
    float acumm = 0.0;
    for(int i = 0; i < sz; i++){
        acumm += vec1[i]*vec2[i];
    }
    return (int) acumm;
}

void SwapRows(float ** matrix, int row1, int row2, int ncols){
    for(int i = 0; i < ncols; i++){
        float tmp = matrix[row1][i];
        matrix[row1][i] = matrix[row2][i];
        matrix[row2][i] = tmp; 
    }
}

template <class T>
void SwapVector(T * av, int pos1, int pos2){
    T tmp = av[pos1];
    av[pos1] = av[pos2];
    av[pos2] = tmp;
}

int GetMaxRowItem(float ** matrix, int currCol, int nrows){
    int maxRow = currCol;
    for(int i = currCol + 1; i < nrows; i++){
        maxRow = (abs(matrix[i][currCol]) > abs(matrix[maxRow][currCol])) ? i : maxRow;
    }
    return maxRow;
}
void SolveTriangularUpperMatrix(float ** matrix, float * X, int nrows, int ncols, float * ans){
    //float * ans = new float[nrows];
    for(int i = nrows - 1; i >= 0; i--){
        float sum = X[i];
        for(int k = i + 1; k < ncols; k++){
            sum -= matrix[i][k]*ans[k];
        }
        ans[i] = sum/matrix[i][i];
    }
    //return ans;
}
template <class T>
float * SolveTriangularLowerMatrix(float ** matrix, T * X, int nrows, int ncols){
    float * ans = new float[nrows];
    for(int i = 0; i < nrows; i++){
        float sum = (float)X[i];
        for(int k = i - 1; k >= 0; k--){
            sum -= matrix[i][k]*ans[k];
        }
        ans[i] = sum/matrix[i][i];
    }
    return ans;
}
void PartialPivotGauss(float ** matrix, int * X, int nrows, int ncols, float * solution){
    int changearr[nrows];
    float ** Lmat = new float*[nrows];
    for(int p = 0; p < nrows; p++){
        Lmat[p] = new float[ncols];
        memset(Lmat[p], 0.0, ncols*sizeof (float));
    }
    //for(int k = 0; k < nrows; k++)
    //    Lmat[k][k] = 1.0;

    for(int i = 0; i < nrows; i++){
        int maxR = GetMaxRowItem(matrix, i, nrows);
        changearr[i] = maxR;
        if(maxR > i){
            SwapRows(matrix, i, maxR, ncols);
            SwapVector<int>(X, i, maxR);
            SwapRows(Lmat, i, maxR, ncols);
            // Save Permutation
        }
        // Avoid division by 0;
        if(matrix[i][i] > 0 || matrix[i][i] < 0){
            // Start Elimination in current col, from the item in diagonal
            // to the end.
            for(int j = i + 1; j < nrows; j++){
                float mm = - matrix[j][i]/matrix[i][i];
                matrix[j][i] = 0.0;
                for(int k = i+1; k < ncols; k++){
                    matrix[j][k] += matrix[i][k]*mm;
                }
                //X[j] += X[i]*mm;
                Lmat[j][i] = -mm;
            }
        }
        //std::cout<<"**********************"<<std::endl;
        //std::cout<<"Iteracion: "<<i<< ": matrix U:\n";
        //printm(matrix, nrows, ncols);
        //std::cout<<std::endl<<std::endl<<"Matrix L\n";
        //printm(Lmat, nrows, ncols) ;   
        //std::cout<<std::endl<<std::endl<<"Augment\n";
        //printvector(X, nrows);
    }
    for(int i = 0; i < nrows; i++)
        Lmat[i][i] = 1.0;
    for(int i = 0; i < nrows; i++){
        if(changearr[i] > i){
            float tmp = X[changearr[i]];
            //X[changearr[i]] = X[i];
            //X[i] = tmp; 
        }
    }
    // The matrix is returned.
    float * solvematrix = SolveTriangularLowerMatrix(Lmat, X, nrows, ncols);
    //std::cout <<"Solve L matrix\n";
    //printvector(solvematrix, nrows);
    SolveTriangularUpperMatrix(matrix, solvematrix, nrows, ncols, solution);
    //float **mat = MultiplyMatrixes(Lmat, matrix, nrows, ncols, nrows);
    
    //printm(mat, nrows, ncols);
    delete [] solvematrix;
    //return solution;
}
void ComputeBilinearCoeff(int * X1, int * Y1, int * X2, int * Y2, float * coefX, float * coefY, int nPairpoints){
    //Four points is espected
    //float * xTransform = new float[nPairpoints];
    float ** TransformX = new float * [nPairpoints];
    float ** TransformY = new float * [nPairpoints];
    for(int i = 0; i < nPairpoints; i++){
        TransformX[i] = new float[nPairpoints];
        TransformY[i] = new float[nPairpoints];
    }
    //Uses the same matrix of transformations
    for(int i = 0; i < nPairpoints; i++){
        TransformX[i][0] = X1[i];
        TransformX[i][1] = (float)Y1[i];
        TransformX[i][2] = (float)X1[i]*(float)Y1[i];
        TransformX[i][3] = 1.0;

        TransformY[i][0] = X1[i];
        TransformY[i][1] = (float)Y1[i];
        TransformY[i][2] = (float)X1[i]*(float)Y1[i];
        TransformY[i][3] = 1.0;
    }
    //printm(TransformX, 4, 4);
    //printm(TransformY, 4, 4);
    int * Bx = new int[nPairpoints];
    int * By = new int[nPairpoints];
    //std::cout<<"X20 "<<X2[0]<<" X21 "<<X2[1]<<std::endl;
    //
    //std::cout<<"B 0 "<<Bx[0]<<std::endl;
    Bx[0] = X2[0]; Bx[1] = X2[1]; Bx[2] = X2[2]; Bx[3] = X2[3];
    By[0] = Y2[0]; By[1] = Y2[1]; By[2] = Y2[2]; By[3] = Y2[3];

    PartialPivotGauss(TransformX, Bx, nPairpoints, nPairpoints, coefX);
    PartialPivotGauss(TransformY, By, nPairpoints, nPairpoints, coefY);
    //printvector(coefX, 4);
    //printvector(Bx, 4);
    //printvector(By, 4);
    
    delete [] Bx;
    delete [] By;
    // Clean Transform X & Transform Y
    for(int i = 0; i < nPairpoints; i++){
        delete [] TransformX[i];
        delete [] TransformY[i];
    }
    delete [] TransformX;
    delete [] TransformY;
    // Coefficients X:
    /*std::cout<<" Coefficients X:\n";
    printvector(coefX, nPairpoints);
    std::cout<<" Coefficients Y:\n";
    printvector(coefY, nPairpoints);
    */
}

unsigned char * Bilinear(unsigned char * data, unsigned char * BilinearData, int * X1, int * Y1, int * X2, int * Y2, int w, int h){
    float * coefX = new float[4];
    float * coefY = new float[4];
    auto t1 = std::chrono::high_resolution_clock::now();
    ComputeBilinearCoeff(X1, Y1, X2, Y2, coefX, coefY, 4);
    auto t2 = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    std::cout<<"Duration> "<<duration<<std::endl;
    //unsigned char * BilinearData = new unsigned char[w*h*3];
    memset(BilinearData, 0, w*h*3);
    for(int i = 0; i < w; i++){
        for(int j = 0; j < h; j++){
            float d[4] = {(float)i, (float)j, (float)i*j, 1.0};
            int xpos = DotProduct(coefX, d, 4);
            int ypos = DotProduct(coefY, d, 4);
            //std::cout<<"Map->"<<" ("<<i<<", "<<j<<") --> "<<"( "<<xpos<<", "<<ypos<<")"<<std::endl;
            BilinearData[xpos*3 + ypos*w*3] = data[i*3 + w*j*3];
            BilinearData[xpos*3 + ypos*w*3 + 1] = data[i*3 + w*j*3 + 1];
            BilinearData[xpos*3 + ypos*w*3 + 2] = data[i*3 + w*j*3 + 2];
        }
    }
    delete [] coefX;
    delete [] coefY;
    std::cout<<"Finish mapping"<<std::endl;
    //printvector(coefX, 4);
    //printvector(coefY, 4);
    //return BilinearData;
}
/*
int main(){

    float a[3][3] = {{1.0, 3.0, 5.0}, {6.0, 3.0,3.0}, {3.0, 1.0, 1.0}};
    float ** m = new float*[3];
    float * X = new float[3];
    X[0] = 29.4; 
    X[1] = 20.4;
    X[2] = 7.0;
    for( int i = 0; i < 3; i++){
        m[i] = new float[3];
    }
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            m[i][j] = a[i][j];
        }
    }
    printm(m, 3, 3);
    float * solution = PartialPivotGauss(m, X, 3, 3);
    std::cout<<std::endl<<std::endl;
    std::cout<<std::endl<<std::endl;
    for(int i = 0; i < 3; i++){
        std::cout<<X[i]<<"\t";
    }
    std::cout<<std::endl<<std::endl;
    for(int i = 0; i < 3; i++){
        std::cout<<solution[i]<<"\t";
    }
    std::cout<<std::endl;

    unsigned char * X1 = new unsigned char [4];
    unsigned char * Y1 = new unsigned char [4];
    unsigned char * X2 = new unsigned char [4];
    unsigned char * Y2 = new unsigned char [4];


}*/
