#include "Matrix.h"
#include "algorithms.h"

/* Matlab code for household QR
function [Q,R] = myQR(A)
% Compute the QR decomposition of an m-by-n matrix A using
% Householder transformations.
[m,n] = size(A);
Q = eye(m); % Orthogonal transform so far
R = A; % Transformed matrix so far
for j = 1:n
    % -- Find H = I-tau*w*w' to put zeros below R(j,j)
    normx = norm(R(j:end,j));
    s = -sign(R(j,j));
    u1 = R(j,j) - s*normx;
    w = R(j:end,j)/u1;
    w(1) = 1;
    tau = -s*u1/normx;
    % -- R := HR, Q := QH
    R(j:end,:) = R(j:end,:)-(tau*w)*(w'*R(j:end,:));
    Q(:,j:end) = Q(:,j:end)-(Q(:,j:end)*w)*(tau*w)';
end
 */
void QR(float *matrix, float *Q, float *R, int row, int col)
{
	float norm = 0;
	float x, u, tau;
	int sign;
	float w[6];
	float dummy_matrix1[36];
	float dummy_vec[36];
	float dummy_vec2[36];
	float dummy_Q[36];
	int h, i, j, k, l, m;
	//arm_matrix_instance_f32 k1, k2, k3;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
		{
			R[i*col + j] = matrix[i*col  + j]; 
		}
	}
	for (h = 0; h < col; h++)
	{
		for (i = 0; i < row; i++)
		{
			for (j = h; j < row; j++)
			{
				dummy_Q[i*(row-h) + j - h] = Q[i*row  + j]; 
			}
		}
		norm = 0;
		for (k = 0; k < row-h; k++)
		{
			x = R[h*(col+1) + k*col];
			norm += x*x;
		}
		norm  = sqrtf(norm);
		sign = -1;
		if (R[h*(col+1)] < 0)
			sign = 1;
		u = R[h*(col+1)] - sign*norm;
		tau = -sign*u/norm;
		
		for (l = 0; l<row-h; l++)
		{
			w[l+h] = R[h*(col+1) + l*col]/u;
		}
		w[h] = 1;
		for (m = h; m<row; m++)
		{
			dummy_vec2[m] = w[m]*tau;
		}
		
		Matrix_Multiply(w+h, R + h*col, dummy_vec, 1, row-h, col);  
		Matrix_Multiply(dummy_vec2 + h, dummy_vec, dummy_matrix1, row-h, 1, col);
		Matrix_Sub(R+h*col, dummy_matrix1, R+h*col, row-h, col);

		Matrix_Multiply(dummy_Q, w + h, dummy_vec, row, row-h, 1);
		Matrix_Multiply(dummy_vec, dummy_vec2 + h, dummy_matrix1, row, 1, row-h);
		Matrix_Sub(dummy_Q, dummy_matrix1, dummy_Q, row, row-h);
								
		for (i = 0; i < row; i++)
		{
			for (j = 0; j < row-h; j++)
			{
				Q[i*row  + j + h] = dummy_Q[i*(row-h) + j]; 
			}
		}
	}
}


void Matrix_Multiply(float *matrix_1, float *matrix_2, float *output, int row, int com, int col)
{
	int i, j;
	float f = 0;
	float *p, *p1, *p3;
	if (com == 3)
	{
		p = &matrix_1[0];
    p3 = &output[0];
		for (i = 0; i < row; i++)
		{
			p1 = &matrix_2[0];
			for (j = 0; j < col; j++)  
			{
				f = p[0]*p1[0] +p[1]*p1[col] + p[2]*p1[2*col];
				*p3 = f;
        p3++;
				p1++;
			}
			p += com;
		}

	}
	else if (com == 4)
	{
		p = &matrix_1[0];
    p3 = &output[0];
		for (i = 0; i < row; i++)
		{
			p1 = &matrix_2[0];
			for (j = 0; j < col; j++)  
			{
				f = p[0]*p1[0] +p[1]*p1[col] + p[2]*p1[2*col] + p[3]*p1[3*col];
				*p3 = f;
        p3++;
				p1++;
			}
			p += com;
		}
	}
  else if (com == 5)
	{
		p = &matrix_1[0];
    p3 = &output[0];
		for (i = 0; i < row; i++)
		{
			p1 = &matrix_2[0];
			for (j = 0; j < col; j++)  
			{
				f = p[0]*p1[0] +p[1]*p1[col] + p[2]*p1[2*col] + p[3]*p1[3*col] + p[4]*p1[4*col];
				*p3 = f;
        p3++;
				p1++;
			}
			p += com;
		}
	}
  else if (com == 6)
	{
		p = &matrix_1[0];
    p3 = &output[0];
		for (i = 0; i < row; i++)
		{
			p1 = &matrix_2[0];
			for (j = 0; j < col; j++)  
			{
				f = p[0]*p1[0] +p[1]*p1[col] + p[2]*p1[2*col] + p[3]*p1[3*col] + p[4]*p1[4*col] + p[5]*p1[5*col];
				*p3 = f;
        p3++;
				p1++;
			}
			p += com;
		}
	}
  else if (com == 2)
	{
		p = &matrix_1[0];
    p3 = &output[0];
		for (i = 0; i < row; i++)
		{
			p1 = &matrix_2[0];
			for (j = 0; j < col; j++)  
			{
				f = p[0]*p1[0] + p[1]*p1[col];
				*p3 = f;
        p3++;
				p1++;
			}
			p += com;
		}
	}
  else if (com == 1)
	{
		p = &matrix_1[0];
    p3 = &output[0];
		for (i = 0; i < row; i++)
		{
			p1 = &matrix_2[0];
			for (j = 0; j < col; j++)  
			{
				f = p[0]*p1[0];
				*p3 = f;
        p3++;
				p1++;
			}
			p += com;
		}
	}
}

void MatrixInverse(float *ptrIn, float *ptrOut, int size)
{
	float dummy[3*3];
	float dummy2[3*3];
	int i, j, k;
	float f;
	int ii,iii,row;
	for (i=0;i<size*size;i++)
	{
		dummy[i]=ptrIn[i];
		dummy2[i]=0.0;
	}
	j=0;
	for (i=0;i<size;i++)
	{
		dummy2[j]=1.0;
		j+=size+1;
	}
	for (i=1;i<size;i++)  ///forward pass;
	{
		row=i*size;
		for (j=0;j<i;j++)
		{
			ii=row+j;
			iii=j*size+j;
			f=-1**(dummy+ii)/(*(dummy+iii));
			for (k=0;k<size-j;k++)
			{
				*(dummy+ii+k)+=f**(dummy+iii+k);
			}
			for (k=0;k<size;k++)
			{
				*(dummy2+row+k)+=f**(dummy2+j*size+k);
			}
		}
	}
	for (i=size-1;i>-1;i--)   ///backward pass
	{
		row=i*size;
		for (j=size-1;j>i;j--)
		{
			ii=row+j;
			iii=j*size+j;
			f=-1**(dummy+ii)/(*(dummy+iii));
			for (k=0;k<size-j;k++)
			{
				*(dummy+ii-k)+=f**(dummy+iii-k);
			}
			for (k=0;k<size;k++)
			{
				*(dummy2+row+k)+=f**(dummy2+j*size+k);
			}
		}
	}
	for (i=0; i<size ; i++)
	{
		row=i*size;
		for (j=0; j<size; j++)
		{
			*(dummy2+row+j)/=*(dummy+row+i);
		}
	}
	for (i=0;i<size*size;i++)
	{
		ptrOut[i]=dummy2[i];
	}
}

void ColumnConcat(float *matrix_1, float *matrix_2, float *matrix_out, int row, int col_1, int col_2)
{
	int col, i, j;
	col = col_1+col_2;
	for (i = 0; i< row; i++)
	{
		for (j = 0; j<col_1; j++)
		{
			matrix_out[i*col+j] = matrix_1[i*col_1+j];
		}
	}
	for (i = 0; i< row; i++)
	{
		for (j = 0; j<col_2; j++)
		{
			matrix_out[i*col+ j + col_1] = matrix_2[i*col_2 + j];
		}
	}
}

void RowConcat(float *matrix_1, float *matrix_2, float *matrix_out, int row_1, int col, int row_2)
{
	int i, j;
	for (i = 0; i< row_1; i++)
	{
		for (j = 0; j<col; j++)
		{
			matrix_out[i*col+j] = matrix_1[i*col+j];
		}
	}
	for (i = 0; i< row_2; i++)
	{
		for (j = 0; j<col; j++)
		{
			matrix_out[i*col + j + row_1*col] = matrix_2[i*col + j];
		}
	}
}

void Matrixtranspose(float *psrc, float *pout, int srcRow, int srcCol)
{
	int i, j;
	for (i=0; i<srcCol; i++)
	{
		for (j=0; j<srcRow; j++)
		{
			*(pout+i*srcRow+j) = *(psrc+j*srcCol+i);
		}
	}
}

void Matrix_Sub(float *matrix_1, float *matrix_2, float *output, int row, int col)
{
	int i, j;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
		{
			 output[i*col+ j] = matrix_1[i*col+ j] - matrix_2[i*col+ j];
		}
	}
}


