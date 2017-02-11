
#ifndef CURVEREMAP_H
#define CURVEREMAP_H

class CurveRemap{
private:

    // http://codereview.stackexchange.com/a/33043
    byte greatestIndexNotExceeding(unsigned short *arr, unsigned short limit, byte lb, byte ub) {
        byte mid = (lb + ub) / 2;

        // Need to go lower but can't
        if (mid == lb && arr[mid] > limit) {
            return 0;// divergence from original which was -1;
        }

        // Found a candidate, and can't go higher
        if (arr[mid] <= limit && (mid == ub || arr[mid + 1] > limit)) {
            return mid;
        }

        if (arr[mid] <= limit) {
            // Consider upper half
            return greatestIndexNotExceeding(arr, limit, mid + 1, ub);
        } else {
            // Consider lower half
            return greatestIndexNotExceeding(arr, limit, lb, mid);
        }
    };
    
public:    
    unsigned short interp(unsigned short value, unsigned short *x,  unsigned short *y, byte arrayLength){
        byte lb = greatestIndexNotExceeding(x, value, 0, arrayLength-1); // returns lower bound
        byte ub = min(lb+1, arrayLength-1); // upper bound
        float f = lb==ub ? 1.0 : (value-x[lb]) / float(x[ub] - x[lb]);
        //Serial.printf("v %i, lb[%i] %i, ub[%i] %i, f %0.2f, y ", value, lb, x[lb], ub, x[ub], f);
        return y[lb] + (f * (y[ub]-y[lb]));
    };
    
};
#endif // CURVEREMAP_H

/*
http://www.geeksforgeeks.org/the-ubiquitous-binary-search-set-1/
// largest value <= key
// Invariant: A[l] <= key and A[r] > key
// Boundary: |r - l| = 1
// Input: A[l .... r-1]
// Precondition: A[l] <= key <= A[r]
int Floor(int A[], int l, int r, int key)
{
    int m;
 
    while( r - l > 1 )
    {
        m = l + (r - l)/2;
 
        if( A[m] <= key )
            l = m;
        else
            r = m;
    }
 
    return A[l];
}
 
// Initial call
int Floor(int A[], int size, int key)
{
    // Add error checking if key < A[0]
    if( key < A[0] )
        return -1;
 
    // Observe boundaries
    return Floor(A, 0, size, key);
}
*/
