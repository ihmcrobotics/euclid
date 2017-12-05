package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public interface LineSegment2DBasics extends LineSegment2DReadOnly
{
   /**
    * Sets this line segment to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    */
   default void set(LineSegment2DReadOnly other)
   {
      set(other.getFirstEndpoint(), other.getSecondEndpoint());
   }
   
   /**
    * Redefines this line segments with new endpoints.
    *
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    */
   default void set(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      setFirstEndpoint(firstEndpointX, firstEndpointY);
      setSecondEndpoint(secondEndpointX, secondEndpointY);
   }

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    */
   void setFirstEndpoint(double firstEndpointX, double firstEndpointY);

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    */
   default void setFirstEndpoint(Point2DReadOnly firstEndpoint)
   {
      setFirstEndpoint(firstEndpoint.getX(), firstEndpoint.getY());
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    */
   void setSecondEndpoint(double secondEndpointX, double secondEndpointY);

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void setSecondEndpoint(Point2DReadOnly secondEndpoint)
   {
      setSecondEndpoint(secondEndpoint.getX(), secondEndpoint.getY());
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void set(Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param endpoints a two-element array containing in order the first and second endpoints for
    *           this line segment. Not modified.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   default void set(Point2DReadOnly[] endpoints)
   {
      if (endpoints.length != 2)
         throw new RuntimeException("Length of input array is not correct. Length = " + endpoints.length + ", expected an array of two elements");
      set(endpoints[0], endpoints[1]);
   }

   /**
    * Translates this line segment by the given (x, y).
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param x the distance to translate this line along the x-axis.
    * @param y the distance to translate this line along the y-axis.
    */
   default void translate(double x, double y)
   {
      setFirstEndpoint(getFirstEndpointX() + x, getFirstEndpointY() + y);
      setSecondEndpoint(getSecondEndpointX() + x, getSecondEndpointY() + y);
   }

   /**
    * Translates this line segment by the given (x, y) contained in {@code translation}.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param translation the translation to add to each endpoint of this line segment. Not modified.
    */
   default void translate(Tuple2DReadOnly translation)
   {
      translate(translation.getX(), translation.getY());
   }

   /**
    * Swaps this line segment's endpoints.
    */
   default void flipDirection()
   {
      double x = getFirstEndpointX();
      double y = getFirstEndpointY();

      setFirstEndpoint(getSecondEndpoint());
      setSecondEndpoint(x, y);
   }

   void shift(boolean shiftToLeft, double distanceToShift);

   /**
    * Translates this line segment by {@code distanceToShift} along the vector perpendicular to this
    * line segment's direction and pointing to the left.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param distanceToShift the distance to shift this line segment.
    */
   default void shiftToLeft(double distanceToShift)
   {
      shift(true, distanceToShift);
   }

   /**
    * Translates this line segment by {@code distanceToShift} along the vector perpendicular to this
    * line segment's direction and pointing to the right.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param distanceToShift the distance to shift this line segment.
    */
   default void shiftToRight(double distanceToShift)
   {
      shift(false, distanceToShift);
   }
}
