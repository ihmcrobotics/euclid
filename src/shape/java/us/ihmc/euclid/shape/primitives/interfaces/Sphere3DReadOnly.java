package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a sphere 3D.
 * <p>
 * A sphere 3D is represented by its position and radius.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Sphere3DReadOnly extends Shape3DReadOnly
{
   /**
    * Gets the radius of this sphere.
    *
    * @return the value of the radius.
    */
   double getRadius();

   /**
    * Gets the read-only reference of the position of this sphere.
    *
    * @return the position of this sphere.
    */
   Point3DReadOnly getPosition();

   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this sphere.
    * </p>
    */
   @Override
   default Point3DReadOnly getCentroid()
   {
      return getPosition();
   }

   /** {@inheritDoc} */
   @Override
   default double getVolume()
   {
      return EuclidShapeTools.sphereVolume(getRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getPosition().containsNaN() || Double.isNaN(getRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      return EuclidShapeTools.evaluatePoint3DSphere3DCollision(pointToCheck,
                                                               getPosition(),
                                                               getRadius(),
                                                               closestPointOnSurfaceToPack,
                                                               normalAtClosestPointToPack) <= 0.0;
   }

   /** {@inheritDoc} */
   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, getPosition(), getRadius(), supportingVertexToPack);
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return EuclidShapeTools.signedDistanceBetweenPoint3DAndSphere3D(point, getPosition(), getRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideSphere3D(query, getPosition(), getRadius(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidShapeTools.orthogonalProjectionOntoSphere3D(pointToProject, getPosition(), getRadius(), projectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this sphere.
    * <p>
    * In the case the line and this sphere do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 sphere. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this sphere. It is either equal to 0, 1,
    *         or 2.
    */
   default int intersectionWith(Line3DReadOnly line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this sphere.
    * <p>
    * In the case the line and this sphere do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.s
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this sphere. It is either equal to 0, 1,
    *         or 2.
    */
   default int intersectionWith(Point3DReadOnly pointOnLine,
                                Vector3DReadOnly lineDirection,
                                Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {

      double pointOnLineX = pointOnLine.getX() - getPosition().getX();
      double pointOnLineY = pointOnLine.getY() - getPosition().getY();
      double pointOnLineZ = pointOnLine.getZ() - getPosition().getZ();
      double lineDirectionX = lineDirection.getX();
      double lineDirectionY = lineDirection.getY();
      double lineDirectionZ = lineDirection.getZ();
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(getRadius(),
                                                                                              getRadius(),
                                                                                              getRadius(),
                                                                                              pointOnLineX,
                                                                                              pointOnLineY,
                                                                                              pointOnLineZ,
                                                                                              lineDirectionX,
                                                                                              lineDirectionY,
                                                                                              lineDirectionZ,
                                                                                              firstIntersectionToPack,
                                                                                              secondIntersectionToPack);

      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         firstIntersectionToPack.add(getPosition());
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         secondIntersectionToPack.add(getPosition());
      return numberOfIntersections;
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      double minX = getPosition().getX() - getRadius();
      double minY = getPosition().getY() - getRadius();
      double minZ = getPosition().getZ() - getRadius();
      double maxX = getPosition().getX() + getRadius();
      double maxY = getPosition().getY() + getRadius();
      double maxZ = getPosition().getZ() + getRadius();
      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
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
   Sphere3DBasics copy();

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Sphere3DReadOnly))
         return false;
      Sphere3DReadOnly other = (Sphere3DReadOnly) geometry;
      return EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon) && getPosition().epsilonEquals(other.getPosition(), epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two spheres are geometrically similar
    * to an {@code epsilon}.
    *
    * @param geometry the object to compare to. Not modified.
    * @param epsilon  the tolerance of the comparison.
    * @return {@code true} if the two boxes represent the same geometry, {@code false} otherwise.
    */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (!(geometry instanceof Sphere3DReadOnly))
         return false;
      Sphere3DReadOnly other = (Sphere3DReadOnly) geometry;
      return EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon) && getPosition().geometricallyEquals(other.getPosition(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Sphere3DReadOnly))
         return false;
      Sphere3DReadOnly other = (Sphere3DReadOnly) geometry;
      return getPosition().equals(other.getPosition()) && getRadius() == other.getRadius();
   }

   /**
    * Gets a representative {@code String} of {@code sphere3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906]
    * </pre>
    * </p>
    *
    * @param format   the format to use for each number.
    * @param sphere3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   @Override
   default String toString(String format)
   {
      return EuclidShapeIOTools.getSphere3DString(format, this);
   }
}
