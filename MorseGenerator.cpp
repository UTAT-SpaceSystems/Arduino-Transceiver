#include <iostream>
#include <math.h>
#include <fstream>
using namespace std;

int * multiple_morse(int input[], int multiply_num, int length);
void code4morse(int *morse, int length, char letter);
int array2dec(int morse[], int length);
void test();

int main()
{
	ofstream myfile;
	myfile.open("output.txt", std::ofstream::out | std::ofstream::trunc);
	myfile.close();

	int multiply_times = 3;
	int *morse;
	//test();
	
	int space[] = { 0, 0, 0 ,0,0,0,0 };
	morse = multiple_morse(space, multiply_times, sizeof(space) / sizeof(int));
	code4morse(morse, sizeof(space) / sizeof(int)*multiply_times, ' ');

	int A[] = { 1,0,1,1,1};
	morse = multiple_morse(A, multiply_times, sizeof(A) / sizeof(int));
	code4morse(morse, sizeof(A) / sizeof(int)*multiply_times,'A');

	int B[] = { 1,1,1,0,1,0,1,0,1 };
	morse = multiple_morse(B, multiply_times, sizeof(B) / sizeof(int));
	code4morse(morse, sizeof(B) / sizeof(int)*multiply_times, 'B');

	int C[] = {1,1,1,0,1,0,1,1,1,0,1};
	morse = multiple_morse(C, multiply_times, sizeof(C) / sizeof(int));
	code4morse(morse, sizeof(C) / sizeof(int)*multiply_times, 'C');

	int D[] = {1,1,1,0,1,0,1};
	morse = multiple_morse(D, multiply_times, sizeof(D) / sizeof(int));
	code4morse(morse, sizeof(D) / sizeof(int)*multiply_times, 'D');

	int E[] = {1};
	morse = multiple_morse(E, multiply_times, sizeof(E) / sizeof(int));
	code4morse(morse, sizeof(E) / sizeof(int)*multiply_times, 'E');

	int F[] = {1,0,1,0,1,1,1,0,1};
	morse = multiple_morse(F, multiply_times, sizeof(F) / sizeof(int));
	code4morse(morse, sizeof(F) / sizeof(int)*multiply_times, 'F');

	int G[] = {1,1,1,0,1,1,1,0,1};
	morse = multiple_morse(G, multiply_times, sizeof(G) / sizeof(int));
	code4morse(morse, sizeof(G) / sizeof(int)*multiply_times, 'G');

	int H[] = {1,0,1,0,1,0,1};
	morse = multiple_morse(H, multiply_times, sizeof(H) / sizeof(int));
	code4morse(morse, sizeof(H) / sizeof(int)*multiply_times, 'H');

	int I[] = {1,0,1};
	morse = multiple_morse(I, multiply_times, sizeof(I) / sizeof(int));
	code4morse(morse, sizeof(I) / sizeof(int)*multiply_times, 'I');

	int J[] = {1,0,1,1,1,0,1,1,1,0,1,1,1};
	morse = multiple_morse(J, multiply_times, sizeof(J) / sizeof(int));
	code4morse(morse, sizeof(J) / sizeof(int)*multiply_times, 'J');

	int K[] = {1,1,1,0,1,1,1};
	morse = multiple_morse(K, multiply_times, sizeof(K) / sizeof(int));
	code4morse(morse, sizeof(K) / sizeof(int)*multiply_times, 'K');

	int L[] = {1,0,1,1,1,0,1,0,1};
	morse = multiple_morse(L, multiply_times, sizeof(L) / sizeof(int));
	code4morse(morse, sizeof(L) / sizeof(int)*multiply_times, 'L');

	int M[] = {1,1,1,0,1,1,1};
	morse = multiple_morse(M, multiply_times, sizeof(M) / sizeof(int));
	code4morse(morse, sizeof(M) / sizeof(int)*multiply_times, 'M');

	int N[] = {1,1,1,0,1};
	morse = multiple_morse(N, multiply_times, sizeof(N) / sizeof(int));
	code4morse(morse, sizeof(N) / sizeof(int)*multiply_times, 'N');

	int O[] = {1,1,1,0,1,1,1,0,1,1,1};
	morse = multiple_morse(O, multiply_times, sizeof(O) / sizeof(int));
	code4morse(morse, sizeof(O) / sizeof(int)*multiply_times, 'O');

	int P[] = {1,0,1,1,1,0,1,1,1,0,1};
	morse = multiple_morse(P, multiply_times, sizeof(P) / sizeof(int));
	code4morse(morse, sizeof(P) / sizeof(int)*multiply_times, 'P');

	int Q[] = {1,1,1,0,1,1,1,0,1,0,1,1,1};
	morse = multiple_morse(Q, multiply_times, sizeof(Q) / sizeof(int));
	code4morse(morse, sizeof(Q) / sizeof(int)*multiply_times, 'Q');

	int R[] = {1,0,1,1,1,0,1};
	morse = multiple_morse(R, multiply_times, sizeof(R) / sizeof(int));
	code4morse(morse, sizeof(R) / sizeof(int)*multiply_times, 'R');

	int S[] = {1,0,1,0,1};
	morse = multiple_morse(S, multiply_times, sizeof(S) / sizeof(int));
	code4morse(morse, sizeof(S) / sizeof(int)*multiply_times, 'S');

	int T[] = {1,1,1};
	morse = multiple_morse(T, multiply_times, sizeof(T) / sizeof(int));
	code4morse(morse, sizeof(T) / sizeof(int)*multiply_times, 'T');

	int U[] = {1,0,1,0,1,1,1};
	morse = multiple_morse(U, multiply_times, sizeof(U) / sizeof(int));
	code4morse(morse, sizeof(U) / sizeof(int)*multiply_times, 'U');

	int V[] = {1,0,1,0,1,0,1,1,1};
	morse = multiple_morse(V, multiply_times, sizeof(V) / sizeof(int));
	code4morse(morse, sizeof(V) / sizeof(int)*multiply_times, 'V');

	int W[] = {1,0,1,1,1,0,1,1,1};
	morse = multiple_morse(W, multiply_times, sizeof(W) / sizeof(int));
	code4morse(morse, sizeof(W) / sizeof(int)*multiply_times, 'W');

	int X[] = {1,1,1,0,1,0,1,0,1,1,1};
	morse = multiple_morse(X, multiply_times, sizeof(X) / sizeof(int));
	code4morse(morse, sizeof(X) / sizeof(int)*multiply_times, 'X');

	int Y[] = {1,1,1,0,1,0,1,1,1,0,1,1,1};
	morse = multiple_morse(Y, multiply_times, sizeof(Y) / sizeof(int));
	code4morse(morse, sizeof(Y) / sizeof(int)*multiply_times, 'Y');

	int Z[] = {1,1,1,0,1,1,1,0,1,0,1};
	morse = multiple_morse(Z, multiply_times, sizeof(Z) / sizeof(int));
	code4morse(morse, sizeof(Z) / sizeof(int)*multiply_times, 'Z');

	int zero[] = { 1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1};
	morse = multiple_morse(zero, multiply_times, sizeof(zero) / sizeof(int));
	code4morse(morse, sizeof(zero) / sizeof(int)*multiply_times, '0');

	int one[] = { 1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1 };
	morse = multiple_morse(one, multiply_times, sizeof(one) / sizeof(int));
	code4morse(morse, sizeof(one) / sizeof(int)*multiply_times, '1');

	int two[] = { 1,0,1,0,1,1,1,0,1,1,1,0,1,1,1 };
	morse = multiple_morse(two, multiply_times, sizeof(two) / sizeof(int));
	code4morse(morse, sizeof(two) / sizeof(int)*multiply_times, '2');

	int three[] = { 1,0,1,0,1,0,1,1,1,0,1,1,1 };
	morse = multiple_morse(three, multiply_times, sizeof(three) / sizeof(int));
	code4morse(morse, sizeof(three) / sizeof(int)*multiply_times, '3');

	int four[] = { 1,0,1,0,1,0,1,0,1,1,1 };
	morse = multiple_morse(four, multiply_times, sizeof(four) / sizeof(int));
	code4morse(morse, sizeof(four) / sizeof(int)*multiply_times, '4');

	int five[] = { 1,0,1,0,1,0,1,0,1 };
	morse = multiple_morse(five, multiply_times, sizeof(five) / sizeof(int));
	code4morse(morse, sizeof(five) / sizeof(int)*multiply_times, '5');

	int six[] = { 1,1,1,0,1,0,1,0,1,0,1 };
	morse = multiple_morse(six, multiply_times, sizeof(six) / sizeof(int));
	code4morse(morse, sizeof(six) / sizeof(int)*multiply_times, '6');

	int seven[] = { 1,1,1,0,1,1,1,0,1,0,1,0,1 };
	morse = multiple_morse(seven, multiply_times, sizeof(seven) / sizeof(int));
	code4morse(morse, sizeof(seven) / sizeof(int)*multiply_times, '7');

	int eight[] = { 1,1,1,0,1,1,1,0,1,1,1,0,1,0,1 };
	morse = multiple_morse(eight, multiply_times, sizeof(eight) / sizeof(int));
	code4morse(morse, sizeof(eight) / sizeof(int)*multiply_times, '8');

	int nine[] = {1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1};
	morse = multiple_morse(nine, multiply_times, sizeof(nine) / sizeof(int));
	code4morse(morse, sizeof(nine) / sizeof(int)*multiply_times, '9');




	//getchar();
    return 0;
}

//test function
void test() {
	int A[] = { 1,1,1,1 };
	cout << array2dec(A, 4) << endl;
}
//multiply the original morse code by multiply_num
int * multiple_morse(int input[], int multiply_num, int length) {
	
	int * morse;
	morse = new int[length*multiply_num];
	for (int i = 0; i < length; i++) 
		for (int j = 0; j < multiply_num; j++) 
			morse[i*multiply_num +j] = input[i];

	return morse;
}
//write the decimal code for an int array to a file
void code4morse(int morse[], int length,char letter) {
	ofstream myfile;
	myfile.open("output.txt", ios::app);

	int i = length;
	int position = 0;
	myfile << "case '" << letter << "':" << endl;
	for (int j=1; i > 0; i -= 8,j++) {
		if (i < 8) 
			position = i-1;
		else 
			position = 7;

		myfile << "*code" << j << " = " << array2dec(&morse[length - position - 1 - (j - 1) * 8], position + 1) << ";" << endl;
	}
	myfile << "*Length = " << length << ";" << endl << "break;" << endl << endl;

	myfile.close();
}
//convert int array to a decimal value
int array2dec(int *morse, int length) {
	int dec = 0;
	for (int i = 0; i < length; i++) {
		dec += pow(2, length - i - 1)*(*(morse+i));
	}
	return dec;
}

