# How to use the variable `list` in Unity?

## Steps for test a `list`

1. declare a `list` variable `dh = new List<float[]>();`.
1. define a row vector `float[] row1 = { 11.0f, 12.0f, 13.0f, 14.0f };`.
1. add the row vector to list `dh.Add(row1);`.
1. print out messages 'print("dh.Count = " + dh.Count)'.

## Example c# code in Unity

Example c# code in Unity.

```
dh = new List<float[]>();
// Create rows as float arrays and add them to the list directly
float[] row1 = { 11.0f, 12.0f, 13.0f, 14.0f };
float[] row2 = { 21.0f, 22.0f, 23.0f, 24.0f };
float[] row3 = { 31.0f, 32.0f, 33.0f, 34.0f };
float[] row4 = { 41.0f, 42.0f, 43.0f, 44.0f };
// Use Add() method to add rows to the list
dh.Add(row1);
dh.Add(row2);
dh.Add(row3);
dh.Add(row4);
print("teng4test, mdh is defined as: dh.Count = " + dh.Count); //=4
print("teng4test, mdh is defined as: dh[1][1] = " + dh[1][1]); //=22
print("teng4test, mdh is defined as: dh[2][2] = " + dh[2][2]); //=33
print("teng4test, mdh is defined as: dh[3][3] = " + dh[3][3]); //=44
// Print the list values using Debug.Log
print("dh.GetType() = " + dh.GetType()); //= System.Collections.Generic.List`1[System.Single[]]
print("dh[1].GetType() = " + dh[1].GetType()); //= System.Single[].
print("dh[1][1].GetType() = " + dh[1][1].GetType()); //= System.Single
print("dh.Capacity = " + dh.Capacity); //= 4
//print("dh[1].Capacity = " + dh[1].Capacity); //will error: float no capacity.
//print("dh[1][1].Capacity = " + dh[1][1].Capacity); //will error: float no capacity.
foreach (float[] row in dh)
{
   string rowValues = string.Join(", ", row);
   Debug.Log("Row: " + rowValues);
}
```

------
Created on 2024-10-04.
