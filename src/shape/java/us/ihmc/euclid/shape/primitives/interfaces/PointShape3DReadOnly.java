package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a point shape 3D.
 * <p>
 * A point shape 3D is represented by its position.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface PointShape3DReadOnly extends Shape3DReadOnly, Point3DReadOnly
{
   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this point shape.
    * </p>
    */
   @Override
   default Point3DReadOnly getCentroid()
   {
      return this;
   }

   /** {@inheritDoc} */
   @Override
   default double getVolume()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   default double distance(Point3DReadOnly point)
   {
      return Point3DReadOnly.super.distance(point);
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return distance(point);
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Point3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      closestPointOnSurfaceToPack.set(this);
      normalAtClosestPointToPack.sub(pointToCheck, this);
      normalAtClosestPointToPack.normalize();
      return false;
   }
   
   
   

   /**
    * Computes the coordinates of the possible intersections between a line and this point.
    * <p>
    * In the case the line and this point do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 point. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this point. It is either equal to 0, 1.
    */
   default int intersectionWith(Line3DReadOnly line, Point3DBasics firstIntersectionToPack,Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack,secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this point.
    * <p>
    * In the case the line and this point do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.s
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this point. It is either equal to 0, 1.
    */
   default int intersectionWith(Point3DReadOnly pointOnLine,
                                Vector3DReadOnly lineDirection,
                                Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {

      double pointX = getX();
      double pointY = getY();
      double pointZ = getZ();
      double lineStartX = pointOnLine.getX() - getX();
      double lineStartY = pointOnLine.getY() - getY();
      double lineStartZ = pointOnLine.getZ() - getZ();
      double lineDirectionX = lineDirection.getX();
      double lineDirectionY = lineDirection.getY();
      double lineDirectionZ = lineDirection.getZ();
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndPointShape3DImpl(pointX,
                                                                                               pointY,
                                                                                               pointZ,
                                                                                               lineStartX,
                                                                                               lineStartY,
                                                                                               lineStartZ,
                                                                                               lineDirectionX,
                                                                                               lineDirectionY,
                                                                                               lineDirectionZ,
                                                                                               firstIntersectionToPack);

      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         firstIntersectionToPack.add(pointX, pointY, pointZ);

      return numberOfIntersections;
   }
   
   /** {@inheritDoc} */
   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.set(this);
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      projectionToPack.set(this);
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      return distanceSquared(query) <= epsilon * epsilon;
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.set(this, this);
   }

   /** {@inheritDoc} */
   @Override
   default boolean isConvex()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPrimitive()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isDefinedByPose()
   {
      return false;
   }

   /**
    * Returns {@code null} as this shape is not defined by a pose.
    */
   @Override
   default Shape3DPoseReadOnly getPose()
   {
      return null;
   }

   @Override
   PointShape3DBasics copy();

   /**
    * Provides a {@code String} representation of this point shape 3D as follows:
    *
    * <pre>
    * Point shape 3D: (-0.362, -0.617,  0.066 )
    * </pre>
    */
   @Override
   default String toString(String format)
   {
      return EuclidShapeIOTools.getPointShape3DString(format, this);
   }
}
