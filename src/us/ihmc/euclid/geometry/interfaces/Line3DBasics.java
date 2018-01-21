package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Line3DBasics extends Line3DReadOnly
{
   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param pointOnLineZ the new z-coordinate of the point on this line.
    */
   void setPoint(double pointOnLineX, double pointOnLineY, double pointOnLineZ);

   /**
    * Changes the direction of this line by setting it to the normalized values provided.
    *
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @param lineDirectionZ the new z-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   void setDirection(double lineDirectionX, double lineDirectionY, double lineDirectionZ);

   /**
    * Changes the direction of this line by setting it to the raw values provided.
    *
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @param lineDirectionZ the new z-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   void setDirectionUnsafe(double lineDirectionX, double lineDirectionY, double lineDirectionZ);

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    */
   default void setPoint(Point3DReadOnly pointOnLine)
   {
      setPoint(pointOnLine.getX(), pointOnLine.getY(), pointOnLine.getZ());
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   default void setDirection(Vector3DReadOnly lineDirection)
   {
      setDirection(lineDirection.getX(), lineDirection.getY(), lineDirection.getZ());
   }

   default void set(Line3DReadOnly other)
   {
      setPoint(other.getPoint());
      setDirectionUnsafe(other.getDirectionX(), other.getDirectionY(), other.getDirectionZ());
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param pointOnLineZ the new z-coordinate of the point on this line.
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @param lineDirectionZ the new z-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   default void set(double pointOnLineX, double pointOnLineY, double pointOnLineZ, double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      setPoint(pointOnLineX, pointOnLineY, pointOnLineZ);
      setDirection(lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    */
   default void set(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      // checkDistinctPoints
      if (firstPointOnLine.equals(secondPointOnLine))
      {
         throw new RuntimeException("Tried to create a line from two coincidal points");
      }

      setPoint(firstPointOnLine);
      setDirection(secondPointOnLine.getX() - firstPointOnLine.getX(), secondPointOnLine.getY() - firstPointOnLine.getY(),
                   secondPointOnLine.getZ() - firstPointOnLine.getZ());
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   default void set(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
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
   default void set(Point3DReadOnly[] twoPointsOnLine)
   {
      if (twoPointsOnLine.length != 2)
         throw new IllegalArgumentException("Length of input array is not correct. Length = " + twoPointsOnLine.length + ", expected an array of two elements");
      set(twoPointsOnLine[0], twoPointsOnLine[1]);
   }

   /**
    * Translates this line by the given (x, y, z).
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    *
    * @param x the distance to translate this line along the x-axis.
    * @param y the distance to translate this line along the y-axis.
    * @param z the distance to translate this line along the z-axis.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   default void translate(double x, double y, double z)
   {
      checkHasBeenInitialized();
      setPoint(getPointX() + x, getPointY() + y, getPointZ() + z);
   }
}
