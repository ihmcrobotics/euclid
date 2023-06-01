package us.ihmc.euclid.geometry.tools;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * This class provides useful methods to compute intersection between a line 3D and a torus
 * TODO geogebra link / put belonging test in the test class
 * 
 * @author Leo Tessier
 */
public class EuclidGeometryTorusTools
{
   /**
    * <p>
    * This method computes the intersections points between a line 2D parallel to X axis and a spiric
    * section in the plane. A spiric section is defined as the intersection between a torus and a plane
    * 3D parallel to the torus axis. The spiric section centered on the plane origin and aligned with
    * its axis is defined by the equation :
    * </p>
    * <img src="https://latex.codecogs.com/svg.image?(x^2+y^2)^2+ax^2+by^2+c=0"/>
    * <p>
    * Intersection points between the two objects are solving the equation
    * </p>
    * <img src="https://latex.codecogs.com/svg.image?x^4+x^2(2y_0^2+a)+(y_0^4+by_0^2+c)=0 <=>
    * x^4+Bx^2+C=0"/> <br>
    * <br>
    * Where <img src="https://latex.codecogs.com/svg.image?y_0"/> represents the y coordinate of the
    * line. <br>
    * <br>
    * 
    * @param radius                       radius of the circle trajectory
    * @param tubeRadius                   radius of the generator circle
    * @param lineYCoordinate              Y coordinate of the line (the line is parallel to x axis)
    * @param distancePlaneToOriginFrame3D distance between the plan intersecting the original torus and
    *                                     the torus axis
    * @param firstIntersectionToPack      the 2D point in which the first intersection found is stored.
    *                                     Can be null. Modified.
    * @param secondIntersectionToPack     the 2D point in which the second intersection found is
    *                                     stored. Can be null. Modified.
    * @param thirdIntersectionToPack      the 2D point in which the third intersection found is stored.
    *                                     Can be null. Modified.
    * @param fourthIntersectionToPack     the 2D point in which the fourth intersection found is
    *                                     stored. Can be null. Modified.
    */

   public static int intersectionBetweenSpiricSection2DAndLine2DParallelToXAxis(double radius,
                                                                                double tubeRadius,
                                                                                double lineYCoordinate,
                                                                                double distancePlaneToOriginFrame3D,

                                                                                Point2DBasics firstIntersectionToPack,
                                                                                Point2DBasics secondIntersectionToPack,
                                                                                Point2DBasics thirdIntersectionToPack,
                                                                                Point2DBasics fourthIntersectionToPack)
   {
      //When injecting line equation into the spiric equation, we obtain a quadratic equation of the type A*X^4+B*X^2+C=0
      double B = 2 * (Math.pow(lineYCoordinate, 2) + Math.pow(distancePlaneToOriginFrame3D, 2) - Math.pow(tubeRadius, 2) - Math.pow(radius, 2));
      double C = Math.pow(lineYCoordinate, 4)
                 + Math.pow(lineYCoordinate, 2) * 2 * (Math.pow(distancePlaneToOriginFrame3D, 2) - Math.pow(tubeRadius, 2) + Math.pow(radius, 2))
                 + Math.pow(Math.pow(distancePlaneToOriginFrame3D, 2) - Math.pow(tubeRadius, 2) + Math.pow(radius, 2), 2)
                 - 4 * Math.pow(distancePlaneToOriginFrame3D, 2) * Math.pow(radius, 2);

      int numIntersections = 0;

      double delta = Math.pow(B, 2) - 4 * C;

      double firstSolution = 0;
      double secondSolution = 0;
      double thirdSolution = 0;
      double fourthSolution = 0;

      if (delta > 0)
      {

         double firstSquareSolution = (-B + Math.sqrt(delta)) / 2;
         double secondSquareSolution = (-B - Math.sqrt(delta)) / 2;

         if (firstSquareSolution > 0)
         {
            firstSolution = Math.sqrt(firstSquareSolution);
            secondSolution = -1 * Math.sqrt(firstSquareSolution);
            numIntersections += 2;
         }

         else if (firstSquareSolution == 0)
         {
            firstSolution = 0;
            secondSolution = Double.NaN;
            numIntersections += 1;
         }
         else
         {
            firstSolution = Double.NaN;
            secondSolution = Double.NaN;
         }

         if (secondSquareSolution > 0)
         {
            thirdSolution = Math.sqrt(secondSquareSolution);
            fourthSolution = -1 * Math.sqrt(secondSquareSolution);
            numIntersections += 2;
         }

         else if (secondSquareSolution == 0)
         {
            thirdSolution = 0;
            fourthSolution = Double.NaN;
            numIntersections += 1;
         }
         else
         {
            thirdSolution = Double.NaN;
            fourthSolution = Double.NaN;
         }

      }

      else if (delta == 0)
      {
         double squareSolution = -B / 2;

         thirdSolution = Double.NaN;
         fourthSolution = Double.NaN;

         if (squareSolution > 0)
         {
            firstSolution = Math.sqrt(squareSolution);
            secondSolution = -1 * Math.sqrt(squareSolution);
            numIntersections += 2;
         }

         else if (squareSolution == 0)
         {
            firstSolution = Math.sqrt(squareSolution);
            secondSolution = Double.NaN;
            numIntersections += 1;
         }

         else
         {
            firstSolution = Double.NaN;
            secondSolution = Double.NaN;
         }
      }

      else
      { //delta<0, no real solution
         firstSolution = Double.NaN;
         secondSolution = Double.NaN;
         thirdSolution = Double.NaN;
         fourthSolution = Double.NaN;
      }

      if (!Double.isNaN(firstSolution))
      {
         firstIntersectionToPack.set(firstSolution, lineYCoordinate);

      }

      else
      {
         firstIntersectionToPack.setToNaN();
      }

      if (!Double.isNaN(secondSolution))
      {
         secondIntersectionToPack.set(secondSolution, lineYCoordinate);
      }

      else
      {
         secondIntersectionToPack.setToNaN();
      }

      if (!Double.isNaN(thirdSolution))
      {
         thirdIntersectionToPack.set(thirdSolution, lineYCoordinate);
      }

      else
      {
         thirdIntersectionToPack.setToNaN();
      }

      if (!Double.isNaN(fourthSolution))
      {
         fourthIntersectionToPack.set(fourthSolution, lineYCoordinate);
      }

      else
      {
         fourthIntersectionToPack.setToNaN();
      }

      return numIntersections;
   }

   /**
    * This method return ranges in which intersections occurs between the given line 2D and spiric,
    * between the given Y range. Intermediate lines will be generated to diminish the range.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the distance between intermediate lines is too important, intersections might be
    * forgotten. Don't set {@code numberOfLinePerIteration < 10}
    * <li>If the given line slope is too small compared to the problem dimensions, intersections might
    * be forgotten.
    * </ul>
    * </p>
    * 
    * @param radius                       radius of the circle trajectory
    * @param tubeRadius                   radius of the generator circle
    * @param distancePlaneToOriginFrame3D distance between the plan intersecting the original torus and
    *                                     the torus axis
    * @param startPoint                   first point 2D defining the line
    * @param endPoint                     second point 2D defining the line
    * @param minY                         minimum Y coordinate to define intermediate lines
    * @param maxY                         maximum Y coordinate to define intermediate lines
    * @param listIntersectionsPerLine     list containing for each intermediate line the list of X
    *                                     coordinate of the intersection of this line with the given
    *                                     line or the torus ranked by ascending order
    * @param listIntersectionsTypePerLine type of each intersection ranked by ascending order
    *                                     "spiricIntersection" for an intersection of the parallel line
    *                                     with the spiric and "lineIntersection for an intersection of
    *                                     the parallel line with the given line
    * @param linePositionForGivenY        for each parralel line, returns "line inside spiric" if the
    *                                     intersection between this parallel line and the given line is
    *                                     inside the spiric and "line outside spiric" otherwise
    * @param firstIntersectionToPack      the 2D point in which the first intersection found between a
    *                                     parallel line and the spiric is stored. Can be null.
    *                                     Modified.
    * @param secondIntersectionToPack     the 2D point in which the second intersection found between a
    *                                     parallel line and the spiric is stored. Can be null.
    *                                     Modified.
    * @param thirdIntersectionToPack      the 2D point in which the third intersection found between a
    *                                     parallel line and the spiric is stored. Can be null.
    *                                     Modified.
    * @param fourthIntersectionToPack     the 2D point in which the fourth intersection between a
    *                                     parallel line and the spiric found is stored. Can be null.
    *                                     Modified.
    */

   private static void sweepLineMethodForIntersectionBetweenLineAndSpiric(double radius,
                                                                          double tubeRadius,
                                                                          double distancePlaneToOriginFrame3D,
                                                                          Point2D startPoint,
                                                                          Point2D endPoint,
                                                                          double minY,
                                                                          double maxY,
                                                                          ArrayList<ArrayList<Double>> listIntersectionsPerLine,
                                                                          ArrayList<ArrayList<String>> listIntersectionsTypePerLine,
                                                                          ArrayList<String> linePositionForGivenY,
                                                                          Point2DBasics firstIntersectionToPack,
                                                                          Point2DBasics secondIntersectionToPack,
                                                                          Point2DBasics thirdIntersectionToPack,
                                                                          Point2DBasics fourthIntersectionToPack)
   {
      if (startPoint.getY() == endPoint.getY())
         throw new IllegalArgumentException("line must not be perpendicular to torus plane due to the sweep line method");

      int numberOfLinePerIteration = 100;
      double yLine = 0;

      Point2D intersectionWithLine = new Point2D();
      Point2D firstPointOnParallelLine = new Point2D();
      Point2D secondPointOnParallelLine = new Point2D();

      for (int lineNumber = 0; lineNumber < numberOfLinePerIteration; lineNumber++)
      {
         yLine = (maxY - minY) / (numberOfLinePerIteration - 1) * lineNumber + minY;

         firstPointOnParallelLine.set(0, yLine);
         secondPointOnParallelLine.set(1, yLine);

         // Compute all intersections between the parallel line i and the line or the spiric
         EuclidGeometryTools.intersectionBetweenTwoLine2Ds(startPoint, endPoint, firstPointOnParallelLine, secondPointOnParallelLine, intersectionWithLine);

         intersectionBetweenSpiricSection2DAndLine2DParallelToXAxis(radius,
                                                                    tubeRadius,
                                                                    yLine,
                                                                    distancePlaneToOriginFrame3D,

                                                                    firstIntersectionToPack,
                                                                    secondIntersectionToPack,
                                                                    thirdIntersectionToPack,
                                                                    fourthIntersectionToPack);

         // building two lists, the first contains the x abscisse of the intersection points, the other one the nature of each intersection points (lineIntersection if line-line and spiric intersection if line-spiric)

         ArrayList<Double> listIntersectionsLineI = new ArrayList<Double>();
         listIntersectionsLineI.add(firstIntersectionToPack.getX());
         listIntersectionsLineI.add(secondIntersectionToPack.getX());
         listIntersectionsLineI.add(thirdIntersectionToPack.getX());
         listIntersectionsLineI.add(fourthIntersectionToPack.getX());
         listIntersectionsLineI.add(intersectionWithLine.getX());

         ArrayList<String> listIntersectionsTypeLineI = new ArrayList<String>();
         listIntersectionsTypeLineI.add("spiricIntersection");
         listIntersectionsTypeLineI.add("spiricIntersection");
         listIntersectionsTypeLineI.add("spiricIntersection");
         listIntersectionsTypeLineI.add("spiricIntersection");
         listIntersectionsTypeLineI.add("lineIntersection");

         //Sort the lists so that the x-abscisses are in the ascending order

         Double temporaryElement = listIntersectionsLineI.get(0);

         for (int firstIndex = 0; firstIndex < 5; firstIndex++) //
         {
            for (int secondIndex = firstIndex + 1; secondIndex < 5; secondIndex++)
            {
               if (listIntersectionsLineI.get(secondIndex) < listIntersectionsLineI.get(firstIndex))
               {
                  temporaryElement = listIntersectionsLineI.get(secondIndex);
                  listIntersectionsLineI.set(secondIndex, listIntersectionsLineI.get(firstIndex));
                  listIntersectionsLineI.set(firstIndex, temporaryElement);

                  if (listIntersectionsTypeLineI.get(firstIndex) == "lineIntersection")
                  {
                     listIntersectionsTypeLineI.set(firstIndex, "spiricIntersection");
                     listIntersectionsTypeLineI.set(secondIndex, "lineIntersection");
                  }
                  else if (listIntersectionsTypeLineI.get(secondIndex) == "lineIntersection")
                  {

                     listIntersectionsTypeLineI.set(firstIndex, "lineIntersection");
                     listIntersectionsTypeLineI.set(secondIndex, "spiricIntersection");

                  }
               }
            }
         }

         // Make sure that all "NaN" remains at the end of the list
         for (int index = 4; index >= 0; index--)
         {
            if (listIntersectionsLineI.get(index).isNaN())
            {
               listIntersectionsLineI.remove(index);
               listIntersectionsLineI.add(Double.NaN);

               String type = listIntersectionsTypeLineI.remove(index);
               listIntersectionsTypeLineI.add(type);

            }
         }

         //         System.out.println("listIntersectionsLineI" + listIntersectionsLineI);
         //         System.out.println("listIntersectionsTypeLineI" + listIntersectionsTypeLineI);

         listIntersectionsPerLine.add(listIntersectionsLineI);
         listIntersectionsTypePerLine.add(listIntersectionsTypeLineI);

      }

      //Now that we have the intersection points with each horizontal line, we can say for a given y if the intersection between the two lines occurs inside the spiric or not.
      int numberOfIntersectionsOnLine = 0;

      for (int lineIndex = 0; lineIndex < numberOfLinePerIteration; lineIndex++)
      {
         // index of the first occurrence of "NaN' in the list of intersections of a line equals the number of intersection of this line with the other objects of the plane.

         if (listIntersectionsPerLine.get(lineIndex).contains(Double.NaN))
         {
            numberOfIntersectionsOnLine = listIntersectionsPerLine.get(lineIndex).indexOf(Double.NaN);
         }

         else
         {
            numberOfIntersectionsOnLine = 5;
         }

         switch (numberOfIntersectionsOnLine)
         {
            case 1:
               // the horizontal line of given Y only intersects with the line : outside spiric
               linePositionForGivenY.add("line outside spiric");
               break;

            case 2:
               // the horizontal line of given Y only intersects with the line and once with the spiric
               linePositionForGivenY.add("line outside spiric");
               break;
            case 3:
               if (listIntersectionsTypePerLine.get(lineIndex).indexOf("lineIntersection") == 0
                   || listIntersectionsTypePerLine.get(lineIndex).indexOf("lineIntersection") == 2)
               {
                  // If the intersection between the two line occurs before or after all others, the line is outside the spiric for the given Y
                  linePositionForGivenY.add("line outside spiric");
               }
               else
               {
                  //the intersection between the two lines is between two intersections horizontal line-spiric
                  // In that case two possibilities :
                  if (yLine == tubeRadius)
                  {
                     //either the horizontal line lay on the top or the bottom of the spiric (y=+-tubeRadius) and the intersection with the line is outside the spiric
                     linePositionForGivenY.add("line outside spiric");
                  }
                  else
                  {
                     //or y<tubeRadius and the intersection with the line is in the spiric
                     linePositionForGivenY.add("line inside spiric");
                  }
               }
               break;

            case 4:
               if (listIntersectionsTypePerLine.get(lineIndex).indexOf("lineIntersection") == 0
                   || listIntersectionsTypePerLine.get(lineIndex).indexOf("lineIntersection") == 3)
               {
                  // If the intersection between the two line occurs before or after all others, the line is outside the spiric for the given Y
                  linePositionForGivenY.add("line outside spiric");
               }
               else
               {
                  //In all other cases the intersection between the two lines occurs in the spiric
                  linePositionForGivenY.add("line inside spiric");
               }
               break;

            case 5:

               if (listIntersectionsTypePerLine.get(lineIndex).indexOf("lineIntersection") == 0
                   || listIntersectionsTypePerLine.get(lineIndex).indexOf("lineIntersection") == 4)
               {
                  // If the intersection between the two line occurs before or after all others, the line is outside the spiric for the given Y
                  linePositionForGivenY.add("line outside spiric");
               }

               else if (listIntersectionsTypePerLine.get(lineIndex).indexOf("lineIntersection") == 1
                        || listIntersectionsTypePerLine.get(lineIndex).indexOf("lineIntersection") == 3)
               {
                  linePositionForGivenY.add("line inside spiric");
               }
               else
               {
                  //intersection between the two line on index two : in the spiric only if the torus is a spindle torus (tubeRadius>radius)
                  if (tubeRadius > radius)
                  {
                     // case spindle torus
                     linePositionForGivenY.add("line inside spiric");

                  }
                  else
                  {
                     linePositionForGivenY.add("line outside spiric");
                  }
               }
               break;
         }

      }
   }

   /**
    * This method return the intersection points between a random line 2D and a spiric section in the
    * plane. The solution is given using the sweep line method adapted to the problem.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the given line slope is too small compared to the problem dimensions, intersections might
    * be forgotten.
    * </ul>
    * </p>
    * 
    * @param radius                       radius of the circle trajectory
    * @param tubeRadius                   radius of the generator circle
    * @param distancePlaneToOriginFrame3D distance between the plan intersecting the original torus and
    *                                     the torus axis
    * @param startPoint                   first point 2D defining the line
    * @param endPoint                     second point 2D defining the line
    * @param firstIntersectionToPack      the 2D point in which the first intersection found between a
    *                                     parallel line and the spiric is stored. Can be null.
    *                                     Modified.
    * @param secondIntersectionToPack     the 2D point in which the second intersection found between a
    *                                     parallel line and the spiric is stored. Can be null.
    *                                     Modified.
    * @param thirdIntersectionToPack      the 2D point in which the third intersection found between a
    *                                     parallel line and the spiric is stored. Can be null.
    *                                     Modified.
    * @param fourthIntersectionToPack     the 2D point in which the fourth intersection between a
    *                                     parallel line and the spiric found is stored. Can be null.
    *                                     Modified.
    */

   public static int intersectionBetweenSpiricSection2DAndLine2D(double radius,
                                                                 double tubeRadius,
                                                                 double distancePlaneToOriginFrame3D,

                                                                 Point2D startPoint,
                                                                 Point2D endPoint,

                                                                 Point2DBasics firstIntersectionToPack,
                                                                 Point2DBasics secondIntersectionToPack,
                                                                 Point2DBasics thirdIntersectionToPack,
                                                                 Point2DBasics fourthIntersectionToPack)
   {

      int iteration = 0;

      ArrayList<ArrayList<Double>> listIntersectionsPerLine = new ArrayList<ArrayList<Double>>();
      ArrayList<ArrayList<String>> listIntersectionsTypePerLine = new ArrayList<ArrayList<String>>();
      ArrayList<String> linePositionForGivenY = new ArrayList<String>();

      sweepLineMethodForIntersectionBetweenLineAndSpiric(radius,
                                                         tubeRadius,
                                                         distancePlaneToOriginFrame3D,
                                                         startPoint,
                                                         endPoint,
                                                         -tubeRadius - 1,
                                                         tubeRadius + 1,
                                                         listIntersectionsPerLine,
                                                         listIntersectionsTypePerLine,
                                                         linePositionForGivenY,
                                                         firstIntersectionToPack,
                                                         secondIntersectionToPack,
                                                         thirdIntersectionToPack,
                                                         fourthIntersectionToPack);

      int numberOfIntersections = 0;
      ArrayList<Point2D> listIntersection = new ArrayList<Point2D>();
      int numberOfLinePerIteration = listIntersectionsPerLine.size();

      for (int lineIndex = 0; lineIndex < listIntersectionsPerLine.size() - 1; lineIndex++) // stops at second to last index because comparison with next index in the following if condition
      {

         if (linePositionForGivenY.get(lineIndex) != linePositionForGivenY.get(lineIndex + 1)) // for each intersection detected (between 0 and 4)
         {

            ArrayList<ArrayList<Double>> newListIntersectionsPerLine = new ArrayList<ArrayList<Double>>();
            ArrayList<ArrayList<String>> newListIntersectionsTypePerLine = new ArrayList<ArrayList<String>>();
            ArrayList<String> newLinePositionForGivenY = new ArrayList<String>();

            numberOfIntersections += 1;

            double minY = (2 * (tubeRadius + 1)) / (numberOfLinePerIteration - 1) * lineIndex - tubeRadius - 1;
            double maxY = (2 * (tubeRadius + 1)) / (numberOfLinePerIteration - 1) * (lineIndex + 1) - tubeRadius - 1;
            double newMinY = 0.0;
            double newMaxY = 0.0;

            while (Math.abs(maxY - minY) > 1e-12 && iteration < 10000)
            {//while precision or convergence non reached 
               iteration += 1;

               sweepLineMethodForIntersectionBetweenLineAndSpiric(radius,
                                                                  tubeRadius,
                                                                  distancePlaneToOriginFrame3D,
                                                                  startPoint,
                                                                  endPoint,
                                                                  minY,
                                                                  maxY,
                                                                  newListIntersectionsPerLine,
                                                                  newListIntersectionsTypePerLine,
                                                                  newLinePositionForGivenY,
                                                                  firstIntersectionToPack,
                                                                  secondIntersectionToPack,
                                                                  thirdIntersectionToPack,
                                                                  fourthIntersectionToPack);

               for (int newLineIndex = 0; newLineIndex < numberOfLinePerIteration - 1; newLineIndex++)
               {
                  if (newLinePositionForGivenY.get(newLineIndex) != newLinePositionForGivenY.get(newLineIndex + 1)) // exactly one intersection must be detected here
                  {

                     newMinY = (maxY - minY) / (numberOfLinePerIteration - 1) * newLineIndex + minY;
                     newMaxY = (maxY - minY) / (numberOfLinePerIteration - 1) * (newLineIndex + 1) + minY;
                     minY = newMinY;
                     maxY = newMaxY;
                  }

               }

               if (numberOfIntersections == 1)
               {
               }
            }
            double YIntersectioni = (maxY + minY) / 2;
            double XIntersectioni = 0.0;

            if (endPoint.getX() == startPoint.getX())
            {
               // if line vertical
               XIntersectioni = endPoint.getX();
            }
            else
            {
               double slope = (endPoint.getY() - startPoint.getY()) / (endPoint.getX() - startPoint.getX());
               double constant = (startPoint.getY() * endPoint.getX() - startPoint.getX() * endPoint.getY()) / (endPoint.getX() - startPoint.getX());
               XIntersectioni = (YIntersectioni - constant) / slope;
            }

            Point2D Intersectioni = new Point2D();
            Intersectioni.set(XIntersectioni, YIntersectioni);
            listIntersection.add(Intersectioni);
            iteration = 0;
         }
      }
      switch (listIntersection.size())
      {
         case 0:
            firstIntersectionToPack.setToNaN();
            secondIntersectionToPack.setToNaN();
            thirdIntersectionToPack.setToNaN();
            fourthIntersectionToPack.setToNaN();
            break;
         case 1:
            firstIntersectionToPack.set(listIntersection.get(0));
            secondIntersectionToPack.setToNaN();
            thirdIntersectionToPack.setToNaN();
            fourthIntersectionToPack.setToNaN();
            break;
         case 2:
            firstIntersectionToPack.set(listIntersection.get(0));
            secondIntersectionToPack.set(listIntersection.get(1));
            thirdIntersectionToPack.setToNaN();
            fourthIntersectionToPack.setToNaN();
            break;
         case 3:
            firstIntersectionToPack.set(listIntersection.get(0));
            secondIntersectionToPack.set(listIntersection.get(1));
            thirdIntersectionToPack.set(listIntersection.get(2));
            fourthIntersectionToPack.setToNaN();
            break;
         case 4:
            firstIntersectionToPack.set(listIntersection.get(0));
            secondIntersectionToPack.set(listIntersection.get(1));
            thirdIntersectionToPack.set(listIntersection.get(2));
            fourthIntersectionToPack.set(listIntersection.get(3));
            break;
      }
      System.out.println("intersection dans le plan :" + listIntersection);

      return numberOfIntersections;
   }

   /**
    * This method returns the intersection between a torus (of axis the axis Y of the frame and with
    * origin the origin of the frame) and a line perpendicular to Z axis.
    * <p>
    * Edge cases:
    * <ul>
    * <li>The line must not be perpendicular to XZ plane (because of the sweep lines).
    * <li>radius must be strictly superior to tube Radius (No spindle torus)
    * </ul>
    * </p>
    * 
    * @param radius                       radius of the circle trajectory
    * @param tubeRadius                   radius of the generator circle
    * @param distancePlaneToOriginFrame3D distance between the plan intersecting the original torus and
    *                                     the torus axis
    * @param startPoint                   first point 3D defining the line
    * @param endPoint                     second point 3D defining the line
    * @param firstIntersectionToPack      the 3D point in which the first intersection found between a
    *                                     parallel line and the spiric is stored. Can be null.
    *                                     Modified.
    * @param secondIntersectionToPack     the 3D point in which the second intersection found between a
    *                                     parallel line and the spiric is stored. Can be null.
    *                                     Modified.
    * @param thirdIntersectionToPack      the 3D point in which the third intersection found between a
    *                                     parallel line and the spiric is stored. Can be null.
    *                                     Modified.
    * @param fourthIntersectionToPack     the 3D point in which the fourth intersection between a
    *                                     parallel line and the spiric found is stored. Can be null.
    *                                     Modified.
    */

   public static int intersectionBetweenTorusAndLine3DInTorusFrame(double radius,
                                                                   double tubeRadius,

                                                                   Point3D startPoint,
                                                                   Point3D endPoint,

                                                                   Point3DBasics firstIntersectionToPack,
                                                                   Point3DBasics secondIntersectionToPack,
                                                                   Point3DBasics thirdIntersectionToPack,
                                                                   Point3DBasics fourthIntersectionToPack)
   {
      if (Math.abs(startPoint.getZ() - endPoint.getZ()) > 1e-8)
      {
         throw new IllegalArgumentException("Line must be perpendicular to z axis");
      }

      double distancePlaneToOriginFrame3D = startPoint.getZ();
      Point2D startPointOnPlane = new Point2D();
      Point2D endPointOnPlane = new Point2D();

      startPointOnPlane.set(startPoint.getX(), startPoint.getY());
      endPointOnPlane.set(endPoint.getX(), endPoint.getY());

      Point2D firstIntersectionOnPlane = new Point2D();
      Point2D secondIntersectionOnPlane = new Point2D();
      Point2D thirdIntersectionOnPlane = new Point2D();
      Point2D fourthIntersectionOnPlane = new Point2D();

      firstIntersectionOnPlane.set(firstIntersectionToPack.getX(), firstIntersectionToPack.getY());
      secondIntersectionOnPlane.set(secondIntersectionToPack.getX(), secondIntersectionToPack.getY());
      thirdIntersectionOnPlane.set(thirdIntersectionToPack.getX(), thirdIntersectionToPack.getY());
      fourthIntersectionOnPlane.set(fourthIntersectionToPack.getX(), fourthIntersectionToPack.getY());

      int numberOfIntersections = intersectionBetweenSpiricSection2DAndLine2D(radius,
                                                                              tubeRadius,
                                                                              distancePlaneToOriginFrame3D,

                                                                              startPointOnPlane,
                                                                              endPointOnPlane,

                                                                              firstIntersectionOnPlane,
                                                                              secondIntersectionOnPlane,
                                                                              thirdIntersectionOnPlane,
                                                                              fourthIntersectionOnPlane);

      if (!firstIntersectionOnPlane.containsNaN())
         firstIntersectionToPack.set(firstIntersectionOnPlane.getX(), firstIntersectionOnPlane.getY(), distancePlaneToOriginFrame3D);
      else
         firstIntersectionToPack.setToNaN();

      if (!secondIntersectionOnPlane.containsNaN())
         secondIntersectionToPack.set(secondIntersectionOnPlane.getX(), secondIntersectionOnPlane.getY(), distancePlaneToOriginFrame3D);
      else
         secondIntersectionToPack.setToNaN();

      if (!thirdIntersectionOnPlane.containsNaN())
         thirdIntersectionToPack.set(thirdIntersectionOnPlane.getX(), thirdIntersectionOnPlane.getY(), distancePlaneToOriginFrame3D);
      else
         thirdIntersectionToPack.setToNaN();

      if (!fourthIntersectionOnPlane.containsNaN())
         fourthIntersectionToPack.set(fourthIntersectionOnPlane.getX(), fourthIntersectionOnPlane.getY(), distancePlaneToOriginFrame3D);
      else
         fourthIntersectionToPack.setToNaN();

      //make sure that intersection points found are on the line and on the torus

      if (!isPoint3DOnLine3D(startPoint, endPoint, firstIntersectionToPack, 1e-6))
         throw new IllegalArgumentException("First point not on the line");
      if (!isPoint3DOnTorus(radius, tubeRadius, firstIntersectionToPack, 1e-8))
         throw new IllegalArgumentException("First point not on the torus");

      if (!isPoint3DOnLine3D(startPoint, endPoint, secondIntersectionToPack, 1e-6))
         throw new IllegalArgumentException("second point not on the line");
      if (!isPoint3DOnTorus(radius, tubeRadius, secondIntersectionToPack, 1e-8))
         throw new IllegalArgumentException("second point not on the torus");

      if (!isPoint3DOnLine3D(startPoint, endPoint, thirdIntersectionToPack, 1e-6))
         throw new IllegalArgumentException("third point not on the line");
      if (!isPoint3DOnTorus(radius, tubeRadius, thirdIntersectionToPack, 1e-8))
         throw new IllegalArgumentException("third point not on the torus");

      if (!isPoint3DOnLine3D(startPoint, endPoint, fourthIntersectionToPack, 1e-6))
         throw new IllegalArgumentException("fourth point not on the line");
      if (!isPoint3DOnTorus(radius, tubeRadius, fourthIntersectionToPack, 1e-8))
         throw new IllegalArgumentException("fourth point not on the torus");

      return numberOfIntersections;
   }

   /**
    * This method return a boolean indicating whether the point {@code intersection} is on the line or
    * not according to the given tolerance.
    * 
    * @param startPoint   first point 3D defining the line
    * @param endPoint     second point 3D defining the line
    * @param intersection point3D which belonging to the line we want to verify
    * @param tolerance    angular tolerance to check collinearity between the vectors defined by
    *                     firsPoint-endPoint and firstPoint-intersection
    */

   private static boolean isPoint3DOnLine3D(Point3D startPoint, Point3D endPoint, Point3DBasics intersection, double tolerance)
   {
      if (intersection.containsNaN())
      {
         return true;
      }

      double lineDirection1x = endPoint.getX() - startPoint.getX();
      double lineDirection1y = endPoint.getY() - startPoint.getY();
      double lineDirection1z = endPoint.getZ() - startPoint.getZ();

      double lineDirection2x = intersection.getX() - startPoint.getX();
      double lineDirection2y = intersection.getY() - startPoint.getY();
      double lineDirection2z = intersection.getZ() - startPoint.getZ();

      return EuclidGeometryTools.areVector3DsParallel(lineDirection1x,
                                                      lineDirection1y,
                                                      lineDirection1z,
                                                      lineDirection2x,
                                                      lineDirection2y,
                                                      lineDirection2z,
                                                      tolerance);
   }

   /**
    * This method return a boolean indicating whether the point {@code intersection} is on the torus or
    * not according to the given tolerance.
    * 
    * @param radius       radius of the circle trajectory
    * @param tubeRadius   radius of the generator circle
    * @param intersection point3D which belonging to the line we want to verify
    * @param tolerance    tolerance
    */
   public static boolean isPoint3DOnTorus(double radius, double tubeRadius, Point3DBasics intersection, double tolerance)
   {
      if (intersection.containsNaN())
      {
         return true;
      }

      double equation = 4 * Math.pow(radius, 2) * (Math.pow(intersection.getX(), 2) + Math.pow(intersection.getZ(), 2))
                        - Math.pow(Math.pow(intersection.getX(), 2) + Math.pow(intersection.getY(), 2) + Math.pow(intersection.getZ(), 2) + Math.pow(radius, 2)
                                   - Math.pow(tubeRadius, 2),
                                   2);
      return equation < tolerance;
   }

}
