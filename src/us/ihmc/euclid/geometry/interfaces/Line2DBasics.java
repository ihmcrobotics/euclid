package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public interface Line2DBasics extends Line2DReadOnly
{
   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   void setDirection(double lineDirectionX, double lineDirectionY);
   
   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    */
   void setPoint(double pointOnLineX, double pointOnLineY);
   
   /**
    * Copies this line and then flips the direction of the copy before returning it.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @throws RuntimeException if this line has not been initialized yet.
    */
   default Line2D negateDirectionCopy()
   {
      checkHasBeenInitialized();
      Line2D ret = new Line2D(this);
      ret.negateDirection();

      return ret;
   }

   /**
    * Applies a counter-clockwise rotation to the direction of this line about the z-axis by
    * {@code angleInRadians}.
    * <p>
    * Note that the point of this line remains unchanged.
    * </p>
    *
    * @param angleInRadians the angle to rotate this line's direction in radians.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   default void rotate(double angleInRadians)
   {
      checkHasBeenInitialized();
      double vXOld = getDirectionX();
      double vYOld = getDirectionY();

      double vXNew = Math.cos(angleInRadians) * vXOld - Math.sin(angleInRadians) * vYOld;
      double vYNew = Math.sin(angleInRadians) * vXOld + Math.cos(angleInRadians) * vYOld;

      setDirection(vXNew, vYNew);
   }
   
   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    */
   default void setPoint(Point2DReadOnly pointOnLine)
   {
      setPoint(pointOnLine.getX(), pointOnLine.getY());
   }
   
   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   default void setDirection(Vector2DReadOnly lineDirection)
   {
      setDirection(lineDirection.getX(), lineDirection.getY());
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param other the other line to copy. Not modified.
    * @throws RuntimeException if the other line has not been initialized yet.
    */
   default void set(Line2DReadOnly other)
   {
      set(other.getPoint(), other.getDirection());
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   default void set(double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      setPoint(pointOnLineX, pointOnLineY);
      setDirection(lineDirectionX, lineDirectionY);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    */
   default void set(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      // checkDistinctPoints
      if (firstPointOnLine.equals(secondPointOnLine))
      {
         throw new RuntimeException("Tried to create a line from two coincidal points.");
      }
      
      setPoint(firstPointOnLine);
      setDirection(secondPointOnLine.getX() - firstPointOnLine.getX(), secondPointOnLine.getY() - firstPointOnLine.getY());
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   default void set(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      setPoint(pointOnLine);
      setDirection(lineDirection);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param twoPointsOnLine a two-element array containing in order the first point and second
    *           point this line is to go through. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   default void set(Point2DReadOnly[] twoPointsOnLine)
   {
      if (twoPointsOnLine.length != 2)
         throw new IllegalArgumentException("Length of input array is not correct. Length = " + twoPointsOnLine.length + ", expected an array of two elements");
      set(twoPointsOnLine[0], twoPointsOnLine[1]);
   }

   void shift(boolean shiftToLeft, double distanceToShift);

   /**
    * Translates this line by {@code distanceToShift} along the vector perpendicular to this line's
    * direction and pointing to the left.
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    *
    * @param distanceToShift the distance to shift this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   default void shiftToLeft(double distanceToShift)
   {
      shift(true, distanceToShift);
   }

   /**
    * Translates this line by {@code distanceToShift} along the vector perpendicular to this line's
    * direction and pointing to the right.
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    *
    * @param distanceToShift the distance to shift this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   default void shiftToRight(double distanceToShift)
   {
      shift(false, distanceToShift);
   }

   /**
    * Translates this line by the given (x, y).
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    *
    * @param x the distance to translate this line along the x-axis.
    * @param y the distance to translate this line along the y-axis.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   default void translate(double x, double y)
   {
      checkHasBeenInitialized();
      setPoint(getPointX() + x, getPointY() + y);
   }
}
