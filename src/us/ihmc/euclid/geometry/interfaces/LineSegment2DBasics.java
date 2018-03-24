package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a line segment 2D.
 * <p>
 * A line segment 2D is a finite-length line defined in the XY-plane by its two 2D endpoints.
 * </p>
 */
public interface LineSegment2DBasics extends LineSegment2DReadOnly, Clearable, Transformable
{
   /**
    * Gets the reference to the first endpoint of this line segment.
    *
    * @return the reference to the first endpoint of this line segment.
    */
   @Override
   Point2DBasics getFirstEndpoint();

   /**
    * Gets the reference to the second endpoint of this line segment.
    *
    * @return the reference to the second endpoint of this line segment.
    */
   @Override
   Point2DBasics getSecondEndpoint();

   /**
    * Tests if this line segment contains {@link Double#NaN}.
    *
    * @return {@code true} if {@link #getFirstEndpoint()} and/or {@link #getSecondEndpoint()} contains
    *         {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return getFirstEndpoint().containsNaN() || getSecondEndpoint().containsNaN();
   }

   /**
    * Sets both endpoints of this line segment to {@link Double#NaN}. After calling this method, this
    * line segment becomes invalid. A new pair of valid endpoints will have to be set so this line
    * segment is again usable.
    */
   @Override
   default void setToNaN()
   {
      getFirstEndpoint().setToNaN();
      getSecondEndpoint().setToNaN();
   }

   /**
    * Sets both endpoints of this line segment to zero.
    */
   @Override
   default void setToZero()
   {
      getFirstEndpoint().setToZero();
      getSecondEndpoint().setToZero();
   }

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
   default void setFirstEndpoint(double firstEndpointX, double firstEndpointY)
   {
      getFirstEndpoint().set(firstEndpointX, firstEndpointY);
   }

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
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    */
   default void setFirstEndpoint(Point3DReadOnly firstEndpoint)
   {
      setFirstEndpoint(firstEndpoint.getX(), firstEndpoint.getY());
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    */
   default void setSecondEndpoint(double secondEndpointX, double secondEndpointY)
   {
      getSecondEndpoint().set(secondEndpointX, secondEndpointY);
   }

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
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void setSecondEndpoint(Point3DReadOnly secondEndpoint)
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
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void set(Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    */
   default void set(Point2DReadOnly firstEndpoint, Vector2DReadOnly fromFirstToSecondEndpoint)
   {
      getFirstEndpoint().set(firstEndpoint);
      getSecondEndpoint().add(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    */
   default void set(Point3DReadOnly firstEndpoint, Vector3DReadOnly fromFirstToSecondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(firstEndpoint);
      getSecondEndpoint().add(fromFirstToSecondEndpoint.getX(), fromFirstToSecondEndpoint.getY());
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

   /**
    * Translates this line segment perpendicularly to its direction.
    *
    * @param shiftToLeft defines to which side this line segment is to be translated.
    * @param distanceToShift the distance this line segment is to be shifted.
    */
   default void shift(boolean shiftToLeft, double distanceToShift)
   {
      double vectorX = getSecondEndpointX() - getFirstEndpointX();
      double vectorY = getSecondEndpointY() - getFirstEndpointY();

      double length = length();
      double orthogonalVectorX = -vectorY / length;
      double orthogonalVectorY = vectorX / length;

      if (!shiftToLeft)
      {
         orthogonalVectorX = -orthogonalVectorX;
         orthogonalVectorY = -orthogonalVectorY;
      }

      orthogonalVectorX = distanceToShift * orthogonalVectorX;
      orthogonalVectorY = distanceToShift * orthogonalVectorY;

      translate(orthogonalVectorX, orthogonalVectorY);
   }

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

   /**
    * Transforms this line segment using the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *            in the XY-plane.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      applyTransform(transform, true);
   }

   /**
    * Transforms this line segment using the inverse of the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *            in the XY-plane.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      applyInverseTransform(transform, true);
   }

   /**
    * Transforms this line segment using the given homogeneous transformation matrix and project the
    * result onto the XY-plane.
    *
    * @param transform the transform to apply on this line segment's endpoints. Not modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of the
    *           given transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation part
    *            of {@code transform} is not a transformation in the XY plane.
    */
   default void applyTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      getFirstEndpoint().applyTransform(transform, checkIfTransformInXYPlane);
      getSecondEndpoint().applyTransform(transform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms this line segment using the given homogeneous transformation matrix and project the
    * result onto the XY-plane.
    *
    * @param transform the transform to apply on this line segment's endpoints. Not modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of the
    *           given transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation part
    *            of {@code transform} is not a transformation in the XY plane.
    */
   default void applyInverseTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      getFirstEndpoint().applyInverseTransform(transform, checkIfTransformInXYPlane);
      getSecondEndpoint().applyInverseTransform(transform, checkIfTransformInXYPlane);
   }
}
