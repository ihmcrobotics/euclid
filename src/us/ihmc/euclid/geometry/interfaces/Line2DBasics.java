package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a line 2D.
 * <p>
 * A line 2D represents an infinitely long line in the XY-plane and defined by a point and a
 * direction.
 * </p>
 */
public interface Line2DBasics extends Line2DReadOnly, Transformable, Clearable
{
   /**
    * Gets the reference to the point through which this line is going.
    *
    * @return the reference to the point.
    */
   @Override
   Point2DBasics getPoint();

   /**
    * Gets the reference to the direction of this line.
    *
    * @return the reference to the direction.
    */
   @Override
   Vector2DBasics getDirection();

   /**
    * Tests if this line contains {@link Double#NaN}.
    *
    * @return {@code true} if {@link #getPoint()} and/or {@link #getDirection()} contains {@link Double#NaN},
    *         {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return getPoint().containsNaN() || getDirection().containsNaN();
   }

   /**
    * Sets the point and vector of this line to zero. After calling this method, this line becomes
    * invalid. A new valid point and valid vector will have to be set so this line is again usable.
    */
   @Override
   default void setToZero()
   {
      getPoint().setToZero();
      getDirection().setToZero();
   }

   /**
    * Sets the point and vector of this line to {@link Double#NaN}. After calling this method, this
    * line becomes invalid. A new valid point and valid vector will have to be set so this line is
    * again usable.
    */
   @Override
   default void setToNaN()
   {
      getPoint().setToNaN();
      getDirection().setToNaN();
   }

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    */
   default void setPoint(double pointOnLineX, double pointOnLineY)
   {
      getPoint().set(pointOnLineX, pointOnLineY);
   }

   /**
    * Changes the direction of this line by setting it to the normalized values provided.
    *
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    */
   default void setDirection(double lineDirectionX, double lineDirectionY)
   {
      getDirection().set(lineDirectionX, lineDirectionY);
      getDirection().normalize();
   }

   /**
    * Flips this line's direction.
    */
   default void negateDirection()
   {
      getDirection().negate();
   }

   /**
    * Applies a counter-clockwise rotation to the direction of this line about the z-axis by
    * {@code angleInRadians}.
    * <p>
    * Note that the point of this line remains unchanged.
    * </p>
    *
    * @param angleInRadians the angle to rotate this line's direction in radians.
    */
   default void rotate(double angleInRadians)
   {
      RotationMatrixTools.applyYawRotation(angleInRadians, getDirection(), getDirection());
   }

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    */
   default void setPoint(Point2DReadOnly pointOnLine)
   {
      getPoint().set(pointOnLine);
   }

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    */
   default void setPoint(Point3DReadOnly pointOnLine)
   {
      getPoint().set(pointOnLine);
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    */
   default void setDirection(Vector2DReadOnly lineDirection)
   {
      getDirection().setAndNormalize(lineDirection);
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    */
   default void setDirection(Vector3DReadOnly lineDirection)
   {
      getDirection().set(lineDirection);
      getDirection().normalize();
   }

   /**
    * Sets this line to be the same as the given line.
    *
    * @param other the other line to copy. Not modified.
    */
   default void set(Line2DReadOnly other)
   {
      getPoint().set(other.getPoint());
      getDirection().set(other.getDirection());
   }

   /**
    * Sets this line to be the same as the given line projected onto the XY-plane.
    *
    * @param line3DReadOnly the line to copy. Not modified.
    */
   default void set(Line3DReadOnly line3DReadOnly)
   {
      getPoint().set(line3DReadOnly.getPoint());
      getDirection().set(line3DReadOnly.getDirection());
   }

   /**
    * Sets this line to go through the endpoints of the given line segment.
    *
    * @param lineSegment2DReadOnly the line segment to copy. Not modified.
    */
   default void set(LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      set(lineSegment2DReadOnly.getFirstEndpoint(), lineSegment2DReadOnly.getSecondEndpoint());
   }

   /**
    * Sets this line to go through the endpoints of the given line segment projected on the XY-plane.
    *
    * @param lineSegment3DReadOnly the line segment to copy. Not modified.
    */
   default void set(LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      set(lineSegment3DReadOnly.getFirstEndpoint(), lineSegment3DReadOnly.getSecondEndpoint());
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
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
    */
   default void set(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      // checkDistinctPoints
      if (firstPointOnLine.equals(secondPointOnLine))
      {
         throw new RuntimeException("Tried to create a line from two coincidal points.");
      }

      setPoint(firstPointOnLine);
      getDirection().sub(secondPointOnLine, firstPointOnLine);
      getDirection().normalize();
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   default void set(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      // checkDistinctPoints
      if (firstPointOnLine.equals(secondPointOnLine))
      {
         throw new RuntimeException("Tried to create a line from two coincidal points.");
      }

      setPoint(firstPointOnLine);
      getDirection().set(secondPointOnLine);
      getDirection().sub(firstPointOnLine.getX(), firstPointOnLine.getY());
      getDirection().normalize();
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   default void set(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      setPoint(pointOnLine);
      setDirection(lineDirection);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   default void set(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      setPoint(pointOnLine);
      setDirection(lineDirection);
   }

   /**
    * Translates this line perpendicularly to its direction.
    *
    * @param shiftToLeft defines to which side this line is to be translated.
    * @param distanceToShift the distance this line is to be shifted.
    */
   default void shift(boolean shiftToLeft, double distanceToShift)
   {
      double vectorX = getDirectionX();
      double vectorY = getDirectionY();

      double orthogonalVectorX = -vectorY;
      double orthogonalVectorY = vectorX;

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
    * Translates this line by {@code distanceToShift} along the vector perpendicular to this line's
    * direction and pointing to the left.
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    *
    * @param distanceToShift the distance to shift this line.
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
    */
   default void translate(double x, double y)
   {
      getPoint().add(x, y);
   }

   /**
    * Transforms this line using the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on this line's point and vector. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *            in the XY-plane.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      applyTransform(transform, true);
   }

   /**
    * Transforms this line using the inverse of the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on this line's point and vector. Not modified.
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
      getPoint().applyTransform(transform, checkIfTransformInXYPlane);
      getDirection().applyTransform(transform, checkIfTransformInXYPlane);
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
      getPoint().applyInverseTransform(transform, checkIfTransformInXYPlane);
      getDirection().applyInverseTransform(transform, checkIfTransformInXYPlane);
   }
}
