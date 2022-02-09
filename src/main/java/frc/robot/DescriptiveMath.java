// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;

/** 
 * This class contains supplemental methods that help with vision processing
 * Some of the code a modified copy of the colt Descriptive package as well as other methods
 */
public class DescriptiveMath {
    /**
 * Returns the winsorized mean of a sorted data sequence.
 *
 * @param sortedData the data sequence; <b>must be sorted ascending</b>.
 * @param mean the mean of the (full) sorted data sequence.
 * @left the number of leading elements to trim.
 * @right the number of trailing elements to trim.
 */
    public static double winsorizedMean(double sortedData[], double mean, int left, int right) {
        int N = sortedData.length;
        if (N == 0)
            throw new IllegalArgumentException("Empty data.");
        if (left + right >= N)
            throw new IllegalArgumentException("Not enough data.");

        double[] sortedElements = sortedData;

        double leftElement = sortedElements[left];
        for (int i = 0; i < left; ++i)
            mean += (leftElement - sortedElements[i]) / N;

        double rightElement = sortedElements[N - 1 - right];
        for (int i = 0; i < right; ++i)
            mean += (rightElement - sortedElements[N - 1 - i]) / N;

        return mean;
    }

    public static double mean(double[] m) {
        double sum = 0;
        for (int i = 0; i < m.length; i++) {
            sum += m[i];
        }
        return sum / m.length;
    }

    public static double trimmean(final double[] arr, final int percent) {
        if ( percent < 0 || 100 < percent ) {
            throw new IllegalArgumentException("Unexpected value: " + percent);
        }
    
        if ( 0 == arr.length ) {
            return Double.NaN;
        }
    
        final int n = arr.length;
        final int k = (int) Math.round(n * ( percent / 100.0 ) / 2.0); // Check overflow
        double list[] = Arrays.copyOf(arr,arr.length);
        Arrays.sort(list);
    
        return winsorizedMean( arr, mean( arr ), k, k );
    }

    // Function to get slice of a primitive array in Java
    public static double[] getSliceOfArray(double[] arr,
            int startIndex, int endIndex) {
        // Get the slice of the Array
        double[] slice = Arrays
                .copyOfRange(
                        // Source
                        arr,
                        startIndex,
                        endIndex);
        return slice;
    }
}
