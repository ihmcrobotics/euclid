package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
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
   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
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

   /**
    * Tests separately and on a per component basis if the pose and the radius of this sphere and
    * {@code other}'s pose and radius are equal to an {@code epsilon}.
    *
    * @param other   the other sphere which pose and radius is to be compared against this radius pose
    *                and radius. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two spheres are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Sphere3DReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon) && getPosition().epsilonEquals(other.getPosition(), epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two spheres are geometrically similar
    * to an {@code epsilon}.
    *
    * @param other   the sphere to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two boxes represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Sphere3DReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon) && getPosition().geometricallyEquals(other.getPosition(), epsilon);
   }

   /**
    * Tests on a per component basis, if this sphere 3D is exactly equal to {@code other}.
    *
    * @param other the other sphere 3D to compare against this. Not modified.
    * @return {@code true} if the two spheres are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(Sphere3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getPosition().equals(other.getPosition()) && getRadius() == other.getRadius();
   }
}
