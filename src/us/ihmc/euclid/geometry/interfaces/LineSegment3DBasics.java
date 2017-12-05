package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public interface LineSegment3DBasics extends LineSegment3DReadOnly
{
   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param firstEndpointZ z-coordinate of the new first endpoint.
    */
   void setFirstEndpoint(double firstEndpointX, double firstEndpointY, double firstEndpointZ);
   
   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @param secondEndpointZ z-coordinate of the new second endpoint.
    */
   void setSecondEndpoint(double secondEndpointX, double secondEndpointY, double secondEndpointZ);

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    */
   default void setFirstEndpoint(Point3DReadOnly firstEndpoint)
   {
      setFirstEndpoint(firstEndpoint.getX(), firstEndpoint.getY(), firstEndpoint.getZ());
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void setSecondEndpoint(Point3DReadOnly secondEndpoint)
   {
      setSecondEndpoint(secondEndpoint.getX(), secondEndpoint.getY(), secondEndpoint.getZ());
   }
   
   /**
    * Sets this line segment to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    */
   default void set(LineSegment3DReadOnly other)
   {
      set(other.getFirstEndpoint(), other.getSecondEndpoint());
   }

   /**
    * Redefines this line segments with new endpoints.
    *
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param firstEndpointZ z-coordinate of the new first endpoint.
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @param secondEndpointZ z-coordinate of the new second endpoint.
    */
   default void set(double firstEndpointX, double firstEndpointY, double firstEndpointZ, double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      setFirstEndpoint(firstEndpointX, firstEndpointY, firstEndpointZ);
      setSecondEndpoint(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void set(Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
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
   default void set(Point3DReadOnly[] endpoints)
   {
      if (endpoints.length != 2)
         throw new RuntimeException("Length of input array is not correct. Length = " + endpoints.length + ", expected an array of two elements");
      set(endpoints[0], endpoints[1]);
   }

   /**
    * Swaps this line segment's endpoints.
    */
   default void flipDirection()
   {
      double x = getFirstEndpointX();
      double y = getFirstEndpointY();
      double z = getFirstEndpointZ();

      setFirstEndpoint(getSecondEndpoint());
      setSecondEndpoint(x, y, z);
   }

   /**
    * Translates this line segment by the given (x, y, z).
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param x the distance to translate this line along the x-axis.
    * @param y the distance to translate this line along the y-axis.
    * @param z the distance to translate this line along the z-axis.
    */
   default void translate(double x, double y, double z)
   {
      setFirstEndpoint(getFirstEndpointX() + x, getFirstEndpointY() + y, getFirstEndpointZ() + z);
      setSecondEndpoint(getSecondEndpointX() + x, getSecondEndpointY() + y, getSecondEndpointZ() + z);
   }

   /**
    * Translates this line segment by the given (x, y, z) contained in {@code translation}.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param translation the translation to add to each endpoint of this line segment. Not modified.
    */
   default void translate(Tuple3DReadOnly translation)
   {
      translate(translation.getX(), translation.getY(), translation.getZ());
   }
}
