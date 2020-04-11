package us.ihmc.euclid.shape.tools;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides a variety of tools for performing operations with shapes.
 *
 * @author Sylvain Bertrand
 */
public class EuclidShapeTools
{
   /**
    * Represents minimum tolerance for triggering edge-case resolution.
    */
   public static final double MIN_DISTANCE_EPSILON = 1.0e-12;

   private EuclidShapeTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Tests whether the {@code query} is located inside an axis-aligned 3D box centered at the origin
    * given its size.
    *
    * @param query     the coordinates of the query. Not modified.
    * @param box3DSize the size of the box. Not modified.
    * @param epsilon   the tolerance to use for this test. A positive value is equivalent to growing
    *                  the size of the box, while a negative value is equivalent to shrinking it.
    * @return {@code true} if the query is inside or on the box's surface, {@code false} otherwise.
    */
   public static boolean isPoint3DInsideBox3D(Point3DReadOnly query, Vector3DReadOnly box3DSize, double epsilon)
   {
      return isPoint3DInsideBox3D(query.getX(), query.getY(), query.getZ(), box3DSize, epsilon);
   }

   /**
    * Tests whether the {@code query} is located inside an axis-aligned 3D box centered at the origin
    * given its size.
    *
    * @param x         the x-coordinates of the query. Not modified.
    * @param y         the y-coordinates of the query. Not modified.
    * @param z         the z-coordinates of the query. Not modified.
    * @param box3DSize the size of the box. Not modified.
    * @param epsilon   the tolerance to use for this test. A positive value is equivalent to growing
    *                  the size of the box, while a negative value is equivalent to shrinking it.
    * @return {@code true} if the query is inside or on the box's surface, {@code false} otherwise.
    */
   public static boolean isPoint3DInsideBox3D(double x, double y, double z, Vector3DReadOnly box3DSize, double epsilon)
   {
      if (Math.abs(x) <= 0.5 * box3DSize.getX() + epsilon)
      {
         if (Math.abs(y) <= 0.5 * box3DSize.getY() + epsilon)
         {
            return Math.abs(z) <= 0.5 * box3DSize.getZ() + epsilon;
         }
      }
      return false;
   }

   /**
    * Computes the distance between the {@code query} and an axis-aligned 3D box centered at the origin
    * given its size.
    * <p>
    * The returned distance is signed as follows:
    * <ul>
    * <li>Positive: the query is located outside the box.
    * <li>Zero: the query is located on the box's surface.
    * <li>Negative: the query is located inside the box.
    * </ul>
    * </p>
    *
    * @param query     the coordinates of the query. Not modified.
    * @param box3DSize the size of the box. Not modified.
    * @return the signed distance between the query and the box.
    */
   public static double signedDistanceBetweenPoint3DAndBox3D(Point3DReadOnly query, Vector3DReadOnly box3DSize)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      boolean isInside = Math.abs(query.getX()) <= halfSizeX && Math.abs(query.getY()) <= halfSizeY && Math.abs(query.getZ()) <= halfSizeZ;

      if (isInside)
      {
         double dx = Math.abs(Math.abs(query.getX()) - halfSizeX);
         double dy = Math.abs(Math.abs(query.getY()) - halfSizeY);
         double dz = Math.abs(Math.abs(query.getZ()) - halfSizeZ);

         return -EuclidCoreTools.min(dx, dy, dz);
      }
      else
      {
         double dx = query.getX() - EuclidCoreTools.clamp(query.getX(), halfSizeX);
         double dy = query.getY() - EuclidCoreTools.clamp(query.getY(), halfSizeY);
         double dz = query.getZ() - EuclidCoreTools.clamp(query.getZ(), halfSizeZ);

         return EuclidCoreTools.norm(dx, dy, dz);
      }
   }

   /**
    * Computes the orthogonal projection of a 3D point on an axis-aligned 3D box centered at the origin
    * given its size.
    * <p>
    * When the query is located inside the box, this method returns {@code false} and
    * {@code projectionToPack} is not modified.
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param box3DSize        the size of the box. Not modified.
    * @param projectionToPack point in which the projection is stored. Modified.
    * @return whether the projection has succeeded or not.
    */
   public static boolean orthogonalProjectionOntoBox3D(Point3DReadOnly pointToProject, Vector3DReadOnly box3DSize, Point3DBasics projectionToPack)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      double xLocal = pointToProject.getX();
      double yLocal = pointToProject.getY();
      double zLocal = pointToProject.getZ();

      if (Math.abs(xLocal) > halfSizeX || Math.abs(yLocal) > halfSizeY || Math.abs(zLocal) > halfSizeZ)
      {
         double xLocalClamped = EuclidCoreTools.clamp(xLocal, halfSizeX);
         double yLocalClamped = EuclidCoreTools.clamp(yLocal, halfSizeY);
         double zLocalClamped = EuclidCoreTools.clamp(zLocal, halfSizeZ);

         projectionToPack.set(xLocalClamped, yLocalClamped, zLocalClamped);

         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * Computes the supporting vertex for an axis-aligned 3D box centered at the origin and given its
    * size.
    * <p>
    * The supporting vertex represents the location on a shape that is the farthest in a given
    * direction.
    * </p>
    *
    * @param supportDirection       the search direction. Not modified.
    * @param box3DSize              the size of the box. Not modified.
    * @param supportingVertexToPack point in which the supporting vertex is stored. Modified.
    */
   public static void supportingVertexBox3D(Vector3DReadOnly supportDirection, Vector3DReadOnly box3DSize, Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.set(supportDirection.getX() > 0.0 ? box3DSize.getX() : -box3DSize.getX(),
                                 supportDirection.getY() > 0.0 ? box3DSize.getY() : -box3DSize.getY(),
                                 supportDirection.getZ() > 0.0 ? box3DSize.getZ() : -box3DSize.getZ());
      supportingVertexToPack.scale(0.5);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given box 3D.
    *
    * @param box3DPosition     the location of the box's center. Not modified.
    * @param box3DOrientation  the orientation of the box. Not modified.
    * @param box3DSize         the size of the box. Not modified.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxBox3D(Point3DReadOnly box3DPosition, RotationMatrixReadOnly box3DOrientation, Vector3DReadOnly box3DSize,
                                       BoundingBox3DBasics boundingBoxToPack)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      double xRange, yRange, zRange;
      if (box3DOrientation.isIdentity())
      {
         xRange = halfSizeX;
         yRange = halfSizeY;
         zRange = halfSizeZ;
      }
      else
      {
         xRange = Math.abs(box3DOrientation.getM00()) * halfSizeX + Math.abs(box3DOrientation.getM01()) * halfSizeY
               + Math.abs(box3DOrientation.getM02()) * halfSizeZ;
         yRange = Math.abs(box3DOrientation.getM10()) * halfSizeX + Math.abs(box3DOrientation.getM11()) * halfSizeY
               + Math.abs(box3DOrientation.getM12()) * halfSizeZ;
         zRange = Math.abs(box3DOrientation.getM20()) * halfSizeX + Math.abs(box3DOrientation.getM21()) * halfSizeY
               + Math.abs(box3DOrientation.getM22()) * halfSizeZ;
      }

      double maxX = box3DPosition.getX() + xRange;
      double maxY = box3DPosition.getY() + yRange;
      double maxZ = box3DPosition.getZ() + zRange;
      double minX = box3DPosition.getX() - xRange;
      double minY = box3DPosition.getY() - yRange;
      double minZ = box3DPosition.getZ() - zRange;

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   /**
    * Evaluates the collision between a 3D point and a 3D box centered at the origin and given its
    * size.
    *
    * @param query                       the location of the query. Not modified.
    * @param box3DSize                   the size of the box. Not modified.
    * @param closestPointOnSurfaceToPack the closest point located on the surface of the box. Modified.
    * @param normalToPack                the surface normal at the closest point. Not modified.
    * @return the signed distance between the query and the box. It is negative when the query is
    *         inside, positive otherwise.
    */
   public static double evaluatePoint3DBox3DCollision(Point3DReadOnly query, Vector3DReadOnly box3DSize, Point3DBasics closestPointOnSurfaceToPack,
                                                      Vector3DBasics normalToPack)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      double dx, dy, dz;
      double xLocal = query.getX();
      double yLocal = query.getY();
      double zLocal = query.getZ();

      boolean isInside = Math.abs(xLocal) <= halfSizeX && Math.abs(yLocal) <= halfSizeY && Math.abs(zLocal) <= halfSizeZ;

      if (isInside)
      {
         dx = Math.abs(Math.abs(xLocal) - halfSizeX);
         dy = Math.abs(Math.abs(yLocal) - halfSizeY);
         dz = Math.abs(Math.abs(zLocal) - halfSizeZ);

         closestPointOnSurfaceToPack.set(xLocal, yLocal, zLocal);

         if (dx < dy)
         {
            if (dx < dz)
               closestPointOnSurfaceToPack.setX(Math.copySign(halfSizeX, xLocal));
            else
               closestPointOnSurfaceToPack.setZ(Math.copySign(halfSizeZ, zLocal));
         }
         else
         {
            if (dy < dz)
               closestPointOnSurfaceToPack.setY(Math.copySign(halfSizeY, yLocal));
            else
               closestPointOnSurfaceToPack.setZ(Math.copySign(halfSizeZ, zLocal));
         }
         normalToPack.setToZero();

         if (dx < dy)
         {
            if (dx < dz)
               normalToPack.setX(Math.copySign(1.0, xLocal));
            else
               normalToPack.setZ(Math.copySign(1.0, zLocal));
         }
         else
         {
            if (dy < dz)
               normalToPack.setY(Math.copySign(1.0, yLocal));
            else
               normalToPack.setZ(Math.copySign(1.0, zLocal));
         }

         return -EuclidCoreTools.min(dx, dy, dz);
      }
      else
      {
         double xLocalClamped = EuclidCoreTools.clamp(xLocal, halfSizeX);
         double yLocalClamped = EuclidCoreTools.clamp(yLocal, halfSizeY);
         double zLocalClamped = EuclidCoreTools.clamp(zLocal, halfSizeZ);

         dx = xLocal - xLocalClamped;
         dy = yLocal - yLocalClamped;
         dz = zLocal - zLocalClamped;

         double distance = EuclidCoreTools.norm(dx, dy, dz);

         closestPointOnSurfaceToPack.set(xLocalClamped, yLocalClamped, zLocalClamped);
         normalToPack.set(dx, dy, dz);
         normalToPack.scale(1.0 / distance);

         return distance;
      }
   }

   /**
    * Tests whether the {@code query} is located inside a 3D capsule.
    *
    * @param query             the coordinates of the query. Not modified.
    * @param capsule3DPosition the coordinates of the capsule's center. Not modified.
    * @param capsule3DAxis     the axis of revolution of the capsule. Not modified.
    * @param capsule3DLength   the length of the capsule, i.e. the distance between the center of the
    *                          half-spheres.
    * @param capsule3DRadius   the radius of the capsule.
    * @param epsilon           the tolerance to use for this test. A positive value is equivalent to
    *                          growing the size of the capsule, while a negative value is equivalent to
    *                          shrinking it.
    * @return {@code true} if the query is inside or on the capsule's surface, {@code false} otherwise.
    */
   public static boolean isPoint3DInsideCapsule3D(Point3DReadOnly query, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                  double capsule3DLength, double capsule3DRadius, double epsilon)
   {
      double capsule3DHalfLength = 0.5 * capsule3DLength;
      double topCenterX = capsule3DPosition.getX() + capsule3DHalfLength * capsule3DAxis.getX();
      double topCenterY = capsule3DPosition.getY() + capsule3DHalfLength * capsule3DAxis.getY();
      double topCenterZ = capsule3DPosition.getZ() + capsule3DHalfLength * capsule3DAxis.getZ();

      double bottomCenterX = capsule3DPosition.getX() - capsule3DHalfLength * capsule3DAxis.getX();
      double bottomCenterY = capsule3DPosition.getY() - capsule3DHalfLength * capsule3DAxis.getY();
      double bottomCenterZ = capsule3DPosition.getZ() - capsule3DHalfLength * capsule3DAxis.getZ();

      double distanceSquared = EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(query.getX(),
                                                                                             query.getY(),
                                                                                             query.getZ(),
                                                                                             topCenterX,
                                                                                             topCenterY,
                                                                                             topCenterZ,
                                                                                             bottomCenterX,
                                                                                             bottomCenterY,
                                                                                             bottomCenterZ);
      double upsizedRadius = capsule3DRadius + epsilon;
      return distanceSquared <= upsizedRadius * upsizedRadius;
   }

   /**
    * Computes the distance between the {@code query} and a 3D capsule.
    * <p>
    * The returned distance is signed as follows:
    * <ul>
    * <li>Positive: the query is located outside the capsule.
    * <li>Zero: the query is located on the capsule's surface.
    * <li>Negative: the query is located inside the capsule.
    * </ul>
    * </p>
    *
    * @param query             the coordinates of the query. Not modified.
    * @param capsule3DPosition the coordinates of the capsule's center. Not modified.
    * @param capsule3DAxis     the axis of revolution of the capsule. Not modified.
    * @param capsule3DLength   the length of the capsule, i.e. the distance between the center of the
    *                          half-spheres.
    * @param capsule3DRadius   the radius of the capsule.
    * @return the signed distance between the query and the capsule.
    */
   public static double signedDistanceBetweenPoint3DAndCapsule3D(Point3DReadOnly query, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                                 double capsule3DLength, double capsule3DRadius)
   {
      double capsuleHalfLength = 0.5 * capsule3DLength;
      double topCenterX = capsule3DPosition.getX() + capsuleHalfLength * capsule3DAxis.getX();
      double topCenterY = capsule3DPosition.getY() + capsuleHalfLength * capsule3DAxis.getY();
      double topCenterZ = capsule3DPosition.getZ() + capsuleHalfLength * capsule3DAxis.getZ();

      double bottomCenterX = capsule3DPosition.getX() - capsuleHalfLength * capsule3DAxis.getX();
      double bottomCenterY = capsule3DPosition.getY() - capsuleHalfLength * capsule3DAxis.getY();
      double bottomCenterZ = capsule3DPosition.getZ() - capsuleHalfLength * capsule3DAxis.getZ();

      double distanceFromAxis = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(query.getX(),
                                                                                       query.getY(),
                                                                                       query.getZ(),
                                                                                       topCenterX,
                                                                                       topCenterY,
                                                                                       topCenterZ,
                                                                                       bottomCenterX,
                                                                                       bottomCenterY,
                                                                                       bottomCenterZ);
      return distanceFromAxis - capsule3DRadius;
   }

   /**
    * Computes the orthogonal projection of a 3D point on a 3D capsule.
    * <p>
    * When the query is located inside the capsule, this method returns {@code false} and
    * {@code projectionToPack} is not modified.
    * </p>
    *
    * @param pointToProject    the point to compute the projection of. Not modified.
    * @param capsule3DPosition the coordinates of the capsule's center. Not modified.
    * @param capsule3DAxis     the axis of revolution of the capsule. Not modified.
    * @param capsule3DLength   the length of the capsule, i.e. the distance between the center of the
    *                          half-spheres.
    * @param capsule3DRadius   the radius of the capsule.
    * @param projectionToPack  point in which the projection is stored. Modified.
    * @return whether the projection has succeeded or not.
    */
   public static boolean orthogonalProjectionOntoCapsule3D(Point3DReadOnly pointToProject, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                           double capsule3DLength, double capsule3DRadius, Point3DBasics projectionToPack)
   {
      if (capsule3DRadius <= 0.0)
      {
         projectionToPack.setToNaN();
         return false;
      }
      else if (capsule3DLength < 0.0)
      {
         projectionToPack.setToNaN();
         return false;
      }
      else if (capsule3DLength == 0.0)
      {
         double distanceSquaredFromCenter = capsule3DPosition.distanceSquared(pointToProject);
         if (distanceSquaredFromCenter <= capsule3DRadius * capsule3DRadius)
            return false;

         projectionToPack.sub(pointToProject, capsule3DPosition);
         projectionToPack.scale(capsule3DRadius / EuclidCoreTools.squareRoot(distanceSquaredFromCenter));
         projectionToPack.add(capsule3DPosition);
         return true;
      }

      double capsule3DHalfLength = 0.5 * capsule3DLength;

      double percentageOnAxis = EuclidGeometryTools.percentageAlongLine3D(pointToProject, capsule3DPosition, capsule3DAxis);

      if (Math.abs(percentageOnAxis) < capsule3DHalfLength)
      {
         double projectionOnAxisX = capsule3DPosition.getX() + percentageOnAxis * capsule3DAxis.getX();
         double projectionOnAxisY = capsule3DPosition.getY() + percentageOnAxis * capsule3DAxis.getY();
         double projectionOnAxisZ = capsule3DPosition.getZ() + percentageOnAxis * capsule3DAxis.getZ();
         double distanceSquaredFromAxis = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(projectionOnAxisX,
                                                                                             projectionOnAxisY,
                                                                                             projectionOnAxisZ,
                                                                                             pointToProject);

         if (distanceSquaredFromAxis <= capsule3DRadius * capsule3DRadius)
            return false;

         projectionToPack.set(pointToProject);
         projectionToPack.sub(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
         projectionToPack.scale(capsule3DRadius / EuclidCoreTools.squareRoot(distanceSquaredFromAxis));
         projectionToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
         return true;
      }
      else if (percentageOnAxis > 0.0)
      {
         double topCenterX = capsule3DPosition.getX() + capsule3DHalfLength * capsule3DAxis.getX();
         double topCenterY = capsule3DPosition.getY() + capsule3DHalfLength * capsule3DAxis.getY();
         double topCenterZ = capsule3DPosition.getZ() + capsule3DHalfLength * capsule3DAxis.getZ();
         double distanceSquaredFromTopCenter = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(topCenterX, topCenterY, topCenterZ, pointToProject);

         if (distanceSquaredFromTopCenter <= capsule3DRadius * capsule3DRadius)
            return false;

         projectionToPack.set(pointToProject);
         projectionToPack.sub(topCenterX, topCenterY, topCenterZ);
         projectionToPack.scale(capsule3DRadius / EuclidCoreTools.squareRoot(distanceSquaredFromTopCenter));
         projectionToPack.add(topCenterX, topCenterY, topCenterZ);
         return true;
      }
      else // if (percentageOnAxis < 0.0)
      {
         double bottomCenterX = capsule3DPosition.getX() - capsule3DHalfLength * capsule3DAxis.getX();
         double bottomCenterY = capsule3DPosition.getY() - capsule3DHalfLength * capsule3DAxis.getY();
         double bottomCenterZ = capsule3DPosition.getZ() - capsule3DHalfLength * capsule3DAxis.getZ();
         double distanceSquaredFromBottomCenter = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(bottomCenterX,
                                                                                                     bottomCenterY,
                                                                                                     bottomCenterZ,
                                                                                                     pointToProject);

         if (distanceSquaredFromBottomCenter <= capsule3DRadius * capsule3DRadius)
            return false;

         projectionToPack.set(pointToProject);
         projectionToPack.sub(bottomCenterX, bottomCenterY, bottomCenterZ);
         projectionToPack.scale(capsule3DRadius / EuclidCoreTools.squareRoot(distanceSquaredFromBottomCenter));
         projectionToPack.add(bottomCenterX, bottomCenterY, bottomCenterZ);
         return true;
      }
   }

   /**
    * Computes the supporting vertex for a 3D capsule.
    * <p>
    * The supporting vertex represents the location on a shape that is the farthest in a given
    * direction.
    * </p>
    *
    * @param supportDirection       the search direction. Not modified.
    * @param capsule3DPosition      the coordinates of the capsule's center. Not modified.
    * @param capsule3DAxis          the axis of revolution of the capsule. Not modified.
    * @param capsule3DLength        the length of the capsule, i.e. the distance between the center of
    *                               the half-spheres.
    * @param capsule3DRadius        the radius of the capsule.
    * @param supportingVertexToPack point in which the supporting vertex is stored. Modified.
    */
   public static void supportingVertexCapsule3D(Vector3DReadOnly supportDirection, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                double capsule3DLength, double capsule3DRadius, Point3DBasics supportingVertexToPack)
   {
      double dot = supportDirection.dot(capsule3DAxis);
      double capsule3DHalfLength = dot > 0.0 ? 0.5 * capsule3DLength : -0.5 * capsule3DLength;

      supportingVertexToPack.setAndScale(capsule3DRadius / supportDirection.length(), supportDirection);
      supportingVertexToPack.add(capsule3DPosition);
      supportingVertexToPack.scaleAdd(capsule3DHalfLength / capsule3DAxis.length(), capsule3DAxis, supportingVertexToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given capsule 3D.
    *
    * @param capsule3DPosition the coordinates of the capsule's center. Not modified.
    * @param capsule3DAxis     the axis of revolution of the capsule. Not modified.
    * @param capsule3DLength   the length of the capsule, i.e. the distance between the center of the
    *                          half-spheres.
    * @param capsule3DRadius   the radius of the capsule.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxCapsule3D(Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis, double capsule3DLength, double capsule3DRadius,
                                           BoundingBox3DBasics boundingBoxToPack)
   {
      double capsule3DHalfLength = 0.5 * capsule3DLength;

      double maxX = Math.abs(capsule3DHalfLength * capsule3DAxis.getX()) + capsule3DRadius;
      double maxY = Math.abs(capsule3DHalfLength * capsule3DAxis.getY()) + capsule3DRadius;
      double maxZ = Math.abs(capsule3DHalfLength * capsule3DAxis.getZ()) + capsule3DRadius;

      double minX = -maxX + capsule3DPosition.getX();
      double minY = -maxY + capsule3DPosition.getY();
      double minZ = -maxZ + capsule3DPosition.getZ();
      maxX += capsule3DPosition.getX();
      maxY += capsule3DPosition.getY();
      maxZ += capsule3DPosition.getZ();

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   /**
    * Evaluates the collision between a 3D point and a 3D capsule.
    *
    * @param query                       the location of the query. Not modified.
    * @param capsule3DPosition           the coordinates of the capsule's center. Not modified.
    * @param capsule3DAxis               the axis of revolution of the capsule. Not modified.
    * @param capsule3DLength             the length of the capsule, i.e. the distance between the
    *                                    center of the half-spheres.
    * @param capsule3DRadius             the radius of the capsule.
    * @param closestPointOnSurfaceToPack the closest point located on the surface of the capsule.
    *                                    Modified.
    * @param normalToPack                the surface normal at the closest point. Not modified.
    * @return the signed distance between the query and the capsule. It is negative when the query is
    *         inside, positive otherwise.
    */
   public static double evaluatePoint3DCapsule3DCollision(Point3DReadOnly query, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                          double capsule3DLength, double capsule3DRadius, Point3DBasics closestPointOnSurfaceToPack,
                                                          Vector3DBasics normalToPack)
   {
      if (capsule3DRadius <= 0.0)
      {
         closestPointOnSurfaceToPack.setToNaN();
         normalToPack.setToNaN();
         return Double.NaN;
      }
      else if (capsule3DLength < 0.0)
      {
         closestPointOnSurfaceToPack.setToNaN();
         normalToPack.setToNaN();
         return Double.NaN;
      }
      else if (capsule3DLength == 0.0)
      {
         return evaluatePoint3DSphere3DCollision(query, capsule3DPosition, capsule3DRadius, closestPointOnSurfaceToPack, normalToPack);
      }

      double capsule3DHalfLength = 0.5 * capsule3DLength;

      double percentageOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, capsule3DPosition, capsule3DAxis);
      if (percentageOnAxis > capsule3DHalfLength)
         percentageOnAxis = capsule3DHalfLength;
      else if (percentageOnAxis < -capsule3DHalfLength)
         percentageOnAxis = -capsule3DHalfLength;

      double projectionOnAxisX = capsule3DPosition.getX() + percentageOnAxis * capsule3DAxis.getX();
      double projectionOnAxisY = capsule3DPosition.getY() + percentageOnAxis * capsule3DAxis.getY();
      double projectionOnAxisZ = capsule3DPosition.getZ() + percentageOnAxis * capsule3DAxis.getZ();
      double distanceSquaredFromAxisClosest = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(projectionOnAxisX,
                                                                                                 projectionOnAxisY,
                                                                                                 projectionOnAxisZ,
                                                                                                 query);

      if (distanceSquaredFromAxisClosest < MIN_DISTANCE_EPSILON)
      { // We need to setup a vector that is orthogonal to the capsule's axis, then we'll perform the projection along that vector.
        // Purposefully picking a large tolerance to ensure sanity of the cross-product.
         if (Math.abs(capsule3DAxis.getY()) > 0.1 || Math.abs(capsule3DAxis.getZ()) > 0.1)
            normalToPack.set(1.0, 0.0, 0.0);
         else
            normalToPack.set(0.0, 1.0, 0.0);

         normalToPack.cross(capsule3DAxis);
         normalToPack.normalize();
         closestPointOnSurfaceToPack.setAndScale(capsule3DRadius, normalToPack);
         closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
         return -capsule3DRadius;
      }
      else
      {
         double distanceFromAxisClosest = EuclidCoreTools.squareRoot(distanceSquaredFromAxisClosest);

         normalToPack.set(query);
         normalToPack.sub(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
         normalToPack.scale(1.0 / distanceFromAxisClosest);

         closestPointOnSurfaceToPack.setAndScale(capsule3DRadius, normalToPack);
         closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);

         return distanceFromAxisClosest - capsule3DRadius;
      }
   }

   /**
    * Tests whether the {@code query} is located inside a 3D capsule.
    *
    * @param query              the coordinates of the query. Not modified.
    * @param cylinder3DPosition the coordinates of the cylinder's center. Not modified.
    * @param cylinder3DAxis     the axis of revolution of the cylinder. Not modified.
    * @param cylinder3DLength   the length of the cylinder.
    * @param cylinder3DRadius   the radius of the cylinder.
    * @param epsilon            the tolerance to use for this test. A positive value is equivalent to
    *                           growing the size of the cylinder, while a negative value is equivalent
    *                           to shrinking it.
    * @return {@code true} if the query is inside or on the cylinder's surface, {@code false}
    *         otherwise.
    */
   public static boolean isPoint3DInsideCylinder3D(Point3DReadOnly query, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                   double cylinder3DLength, double cylinder3DRadius, double epsilon)
   {
      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, cylinder3DPosition, cylinder3DAxis);

      double halfLengthPlusEpsilon = 0.5 * cylinder3DLength + epsilon;

      if (Math.abs(positionOnAxis) > halfLengthPlusEpsilon)
         return false;

      double projectionOnAxisX = cylinder3DPosition.getX() + positionOnAxis * cylinder3DAxis.getX();
      double projectionOnAxisY = cylinder3DPosition.getY() + positionOnAxis * cylinder3DAxis.getY();
      double projectionOnAxisZ = cylinder3DPosition.getZ() + positionOnAxis * cylinder3DAxis.getZ();
      double distanceSquaredFromAxis = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ, query);
      double radiusWithEpsilon = cylinder3DRadius + epsilon;
      return distanceSquaredFromAxis <= radiusWithEpsilon * radiusWithEpsilon;
   }

   /**
    * Computes the distance between the {@code query} and a 3D cylinder.
    * <p>
    * The returned distance is signed as follows:
    * <ul>
    * <li>Positive: the query is located outside the cylinder.
    * <li>Zero: the query is located on the cylinder's surface.
    * <li>Negative: the query is located inside the cylinder.
    * </ul>
    * </p>
    *
    * @param query              the coordinates of the query. Not modified.
    * @param cylinder3DPosition the coordinates of the cylinder's center. Not modified.
    * @param cylinder3DAxis     the axis of revolution of the cylinder. Not modified.
    * @param cylinder3DLength   the length of the cylinder.
    * @param cylinder3DRadius   the radius of the cylinder.
    * @return the signed distance between the query and the cylinder.
    */
   public static double signedDistanceBetweenPoint3DAndCylinder3D(Point3DReadOnly query, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                                  double cylinder3DLength, double cylinder3DRadius)
   {
      if (cylinder3DRadius <= 0.0 || cylinder3DLength <= 0.0)
      {
         return Double.NaN;
      }

      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, cylinder3DPosition, cylinder3DAxis);

      double axisToQueryX = query.getX() - (cylinder3DPosition.getX() + positionOnAxis * cylinder3DAxis.getX());
      double axisToQueryY = query.getY() - (cylinder3DPosition.getY() + positionOnAxis * cylinder3DAxis.getY());
      double axisToQueryZ = query.getZ() - (cylinder3DPosition.getZ() + positionOnAxis * cylinder3DAxis.getZ());
      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(axisToQueryX, axisToQueryY, axisToQueryZ);

      double halfLength = 0.5 * cylinder3DLength;

      if (distanceSquaredFromAxis <= cylinder3DRadius * cylinder3DRadius)
      {
         if (positionOnAxis < -halfLength)
         { // The query is directly below the cylinder
            return -(positionOnAxis + halfLength);
         }

         if (positionOnAxis > halfLength)
         { // The query is directly above the cylinder
            return positionOnAxis - halfLength;
         }

         // The query is inside the cylinder
         double distanceFromAxis = EuclidCoreTools.squareRoot(distanceSquaredFromAxis);
         double dh = halfLength - Math.abs(positionOnAxis);
         double dr = cylinder3DRadius - distanceFromAxis;

         if (dh < dr)
         {
            if (positionOnAxis < 0)
            { // Closer to the bottom face
               return -(positionOnAxis + halfLength);
            }
            else
            { // Closer to the top face
               return positionOnAxis - halfLength;
            }
         }
         else
         { // Closer to the cylinder part
            return distanceFromAxis - cylinder3DRadius;
         }
      }
      else
      { // The query is outside and closest to the cylinder's side.
         double distanceFromAxis = EuclidCoreTools.squareRoot(distanceSquaredFromAxis);

         double positionOnAxisClamped = positionOnAxis;
         if (positionOnAxisClamped < -halfLength)
            positionOnAxisClamped = -halfLength;
         else if (positionOnAxisClamped > halfLength)
            positionOnAxisClamped = halfLength;

         if (positionOnAxisClamped != positionOnAxis)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.

            double projectionOnAxisXClamped = cylinder3DPosition.getX() + positionOnAxisClamped * cylinder3DAxis.getX();
            double projectionOnAxisYClamped = cylinder3DPosition.getY() + positionOnAxisClamped * cylinder3DAxis.getY();
            double projectionOnAxisZClamped = cylinder3DPosition.getZ() + positionOnAxisClamped * cylinder3DAxis.getZ();

            double toCylinderScale = cylinder3DRadius / distanceFromAxis;
            double closestX = axisToQueryX * toCylinderScale + projectionOnAxisXClamped;
            double closestY = axisToQueryY * toCylinderScale + projectionOnAxisYClamped;
            double closestZ = axisToQueryZ * toCylinderScale + projectionOnAxisZClamped;
            return EuclidGeometryTools.distanceBetweenPoint3Ds(closestX, closestY, closestZ, query);
         }
         else
         { // Closest point is on the cylinder.
            return distanceFromAxis - cylinder3DRadius;
         }
      }
   }

   /**
    * Computes the orthogonal projection of a 3D point on a 3D cylinder.
    * <p>
    * When the query is located inside the cylinder, this method returns {@code false} and
    * {@code projectionToPack} is not modified.
    * </p>
    *
    * @param pointToProject     the point to compute the projection of. Not modified.
    * @param cylinder3DPosition the coordinates of the cylinder's center. Not modified.
    * @param cylinder3DAxis     the axis of revolution of the cylinder. Not modified.
    * @param cylinder3DLength   the length of the cylinder.
    * @param cylinder3DRadius   the radius of the cylinder.
    * @param projectionToPack   point in which the projection is stored. Modified.
    * @return whether the projection has succeeded or not.
    */
   public static boolean orthogonalProjectionOntoCylinder3D(Point3DReadOnly pointToProject, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                            double cylinder3DLength, double cylinder3DRadius, Point3DBasics projectionToPack)
   {
      if (cylinder3DRadius <= 0.0 || cylinder3DLength <= 0.0)
      {
         projectionToPack.setToNaN();
         return false;
      }

      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(pointToProject, cylinder3DPosition, cylinder3DAxis);

      double projectionOnAxisX = cylinder3DPosition.getX() + positionOnAxis * cylinder3DAxis.getX();
      double projectionOnAxisY = cylinder3DPosition.getY() + positionOnAxis * cylinder3DAxis.getY();
      double projectionOnAxisZ = cylinder3DPosition.getZ() + positionOnAxis * cylinder3DAxis.getZ();

      double axisToQueryX = pointToProject.getX() - projectionOnAxisX;
      double axisToQueryY = pointToProject.getY() - projectionOnAxisY;
      double axisToQueryZ = pointToProject.getZ() - projectionOnAxisZ;
      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(axisToQueryX, axisToQueryY, axisToQueryZ);

      double halfLength = 0.5 * cylinder3DLength;

      if (distanceSquaredFromAxis <= cylinder3DRadius * cylinder3DRadius)
      {
         if (positionOnAxis < -halfLength)
         { // The query is directly below the cylinder
            projectionToPack.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, pointToProject);
            return true;
         }

         if (positionOnAxis > halfLength)
         { // The query is directly above the cylinder
            projectionToPack.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, pointToProject);
            return true;
         }

         // The query is inside the cylinder
         return false;
      }
      else
      { // The query is outside and closest to the cylinder's side.
         double distanceFromAxis = EuclidCoreTools.squareRoot(distanceSquaredFromAxis);

         double positionOnAxisClamped = positionOnAxis;
         if (positionOnAxisClamped < -halfLength)
            positionOnAxisClamped = -halfLength;
         else if (positionOnAxisClamped > halfLength)
            positionOnAxisClamped = halfLength;

         if (positionOnAxisClamped != positionOnAxis)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.
            double projectionOnAxisXClamped = cylinder3DPosition.getX() + positionOnAxisClamped * cylinder3DAxis.getX();
            double projectionOnAxisYClamped = cylinder3DPosition.getY() + positionOnAxisClamped * cylinder3DAxis.getY();
            double projectionOnAxisZClamped = cylinder3DPosition.getZ() + positionOnAxisClamped * cylinder3DAxis.getZ();

            double toCylinderScale = cylinder3DRadius / distanceFromAxis;
            double closestX = axisToQueryX * toCylinderScale + projectionOnAxisXClamped;
            double closestY = axisToQueryY * toCylinderScale + projectionOnAxisYClamped;
            double closestZ = axisToQueryZ * toCylinderScale + projectionOnAxisZClamped;
            projectionToPack.set(closestX, closestY, closestZ);
            return true;
         }
         else
         { // Closest point is on the cylinder.
            projectionToPack.set(axisToQueryX, axisToQueryY, axisToQueryZ);
            projectionToPack.scale(cylinder3DRadius / distanceFromAxis);
            projectionToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            return true;
         }
      }
   }

   /**
    * Computes the supporting vertex for a 3D cylinder.
    * <p>
    * The supporting vertex represents the location on a shape that is the farthest in a given
    * direction.
    * </p>
    *
    * @param supportDirection       the search direction. Not modified.
    * @param cylinder3DPosition     the coordinates of the cylinder's center. Not modified.
    * @param cylinder3DAxis         the axis of revolution of the cylinder. Not modified.
    * @param cylinder3DLength       the length of the cylinder.
    * @param cylinder3DRadius       the radius of the cylinder.
    * @param supportingVertexToPack point in which the supporting vertex is stored. Modified.
    */
   public static void supportingVertexCylinder3D(Vector3DReadOnly supportDirection, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                 double cylinder3DLength, double cylinder3DRadius, Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.set(supportDirection);

      double axisNormInverse = 1.0 / cylinder3DAxis.length();
      double dot = supportDirection.dot(cylinder3DAxis) * axisNormInverse;
      supportingVertexToPack.setAndScale(dot * axisNormInverse, cylinder3DAxis);
      supportingVertexToPack.sub(supportDirection, supportingVertexToPack);
      double distanceSquaredFromAxis = supportingVertexToPack.distanceFromOriginSquared();

      if (distanceSquaredFromAxis < MIN_DISTANCE_EPSILON)
      {
         supportingVertexToPack.set(cylinder3DPosition);
      }
      else
      {
         supportingVertexToPack.scale(cylinder3DRadius / EuclidCoreTools.squareRoot(distanceSquaredFromAxis));
         supportingVertexToPack.add(cylinder3DPosition);
      }

      double cylinder3DHalfLength = dot > 0.0 ? 0.5 * cylinder3DLength : -0.5 * cylinder3DLength;
      supportingVertexToPack.scaleAdd(cylinder3DHalfLength * axisNormInverse, cylinder3DAxis, supportingVertexToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given cylinder 3D.
    *
    * @param cylinder3DPosition the coordinates of the cylinder's center. Not modified.
    * @param cylinder3DAxis     the axis of revolution of the cylinder. Not modified.
    * @param cylinder3DLength   the length of the cylinder.
    * @param cylinder3DRadius   the radius of the cylinder.
    * @param boundingBoxToPack  the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxCylinder3D(Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis, double cylinder3DLength,
                                            double cylinder3DRadius, BoundingBox3DBasics boundingBoxToPack)
   {
      // From https://iquilezles.org/www/articles/diskbbox/diskbbox.htm
      double cylinder3DHalfLength = 0.5 * cylinder3DLength;

      double invNormSquared = 1.0 / cylinder3DAxis.lengthSquared();
      double capMinMaxX = Math.max(0.0, cylinder3DRadius * EuclidCoreTools.squareRoot(1.0 - cylinder3DAxis.getX() * cylinder3DAxis.getX() * invNormSquared));
      double capMinMaxY = Math.max(0.0, cylinder3DRadius * EuclidCoreTools.squareRoot(1.0 - cylinder3DAxis.getY() * cylinder3DAxis.getY() * invNormSquared));
      double capMinMaxZ = Math.max(0.0, cylinder3DRadius * EuclidCoreTools.squareRoot(1.0 - cylinder3DAxis.getZ() * cylinder3DAxis.getZ() * invNormSquared));

      double maxX = Math.abs(cylinder3DHalfLength * cylinder3DAxis.getX()) + capMinMaxX;
      double maxY = Math.abs(cylinder3DHalfLength * cylinder3DAxis.getY()) + capMinMaxY;
      double maxZ = Math.abs(cylinder3DHalfLength * cylinder3DAxis.getZ()) + capMinMaxZ;

      double minX = -maxX + cylinder3DPosition.getX();
      double minY = -maxY + cylinder3DPosition.getY();
      double minZ = -maxZ + cylinder3DPosition.getZ();
      maxX += cylinder3DPosition.getX();
      maxY += cylinder3DPosition.getY();
      maxZ += cylinder3DPosition.getZ();

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   /**
    * Evaluates the collision between a 3D point and a 3D cylinder.
    *
    * @param query                       the location of the query. Not modified.
    * @param cylinder3DPosition          the coordinates of the cylinder's center. Not modified.
    * @param cylinder3DAxis              the axis of revolution of the cylinder. Not modified.
    * @param cylinder3DLength            the length of the cylinder.
    * @param cylinder3DRadius            the radius of the cylinder.
    * @param closestPointOnSurfaceToPack the closest point located on the surface of the cylinder.
    *                                    Modified.
    * @param normalToPack                the surface normal at the closest point. Not modified.
    * @return the signed distance between the query and the cylinder. It is negative when the query is
    *         inside, positive otherwise.
    */
   public static double evaluatePoint3DCylinder3DCollision(Point3DReadOnly query, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                           double cylinder3DLength, double cylinder3DRadius, Point3DBasics closestPointOnSurfaceToPack,
                                                           Vector3DBasics normalToPack)
   {
      if (cylinder3DRadius <= 0.0 || cylinder3DLength <= 0.0)
      {
         closestPointOnSurfaceToPack.setToNaN();
         normalToPack.setToNaN();
         return Double.NaN;
      }

      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, cylinder3DPosition, cylinder3DAxis);

      double projectionOnAxisX = cylinder3DPosition.getX() + positionOnAxis * cylinder3DAxis.getX();
      double projectionOnAxisY = cylinder3DPosition.getY() + positionOnAxis * cylinder3DAxis.getY();
      double projectionOnAxisZ = cylinder3DPosition.getZ() + positionOnAxis * cylinder3DAxis.getZ();

      double axisToQueryX = query.getX() - projectionOnAxisX;
      double axisToQueryY = query.getY() - projectionOnAxisY;
      double axisToQueryZ = query.getZ() - projectionOnAxisZ;
      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(axisToQueryX, axisToQueryY, axisToQueryZ);

      double halfLength = 0.5 * cylinder3DLength;

      if (distanceSquaredFromAxis <= cylinder3DRadius * cylinder3DRadius)
      {
         if (positionOnAxis < -halfLength)
         { // The query is directly below the cylinder
            closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, query);
            normalToPack.setAndNegate(cylinder3DAxis);
            return -(positionOnAxis + halfLength);
         }

         if (positionOnAxis > halfLength)
         { // The query is directly above the cylinder
            closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, query);
            normalToPack.set(cylinder3DAxis);
            return positionOnAxis - halfLength;
         }

         // The query is inside the cylinder
         double distanceFromAxis = EuclidCoreTools.squareRoot(distanceSquaredFromAxis);
         double dh = halfLength - Math.abs(positionOnAxis);
         double dr = cylinder3DRadius - distanceFromAxis;

         if (dh < dr)
         {
            if (positionOnAxis < 0)
            { // Closer to the bottom face
               closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, query);
               normalToPack.setAndNegate(cylinder3DAxis);
               return -(positionOnAxis + halfLength);
            }
            else
            { // Closer to the top face
               closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, query);
               normalToPack.set(cylinder3DAxis);
               return positionOnAxis - halfLength;
            }
         }
         else if (distanceSquaredFromAxis < MIN_DISTANCE_EPSILON)
         { // Edge-case: the query is too close to the axis to compute sane results
           // We need to setup a vector that is orthogonal to the cylinder's axis, then we'll perform the projection along that vector.
           // Purposefully picking a large tolerance to ensure sanity of the cross-product.
            if (Math.abs(cylinder3DAxis.getY()) > 0.1 || Math.abs(cylinder3DAxis.getZ()) > 0.1)
               normalToPack.set(1.0, 0.0, 0.0);
            else
               normalToPack.set(0.0, 1.0, 0.0);

            normalToPack.cross(cylinder3DAxis);
            normalToPack.normalize();
            closestPointOnSurfaceToPack.setAndScale(cylinder3DRadius, normalToPack);
            closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            return -cylinder3DRadius;
         }
         else
         { // Closer to the cylinder part
            normalToPack.set(axisToQueryX, axisToQueryY, axisToQueryZ);
            normalToPack.scale(1.0 / distanceFromAxis);

            closestPointOnSurfaceToPack.setAndScale(cylinder3DRadius, normalToPack);
            closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);

            return distanceFromAxis - cylinder3DRadius;
         }
      }
      else
      { // The query is outside and closest to the cylinder's side.
         double distanceFromAxis = EuclidCoreTools.squareRoot(distanceSquaredFromAxis);

         double positionOnAxisClamped = positionOnAxis;
         if (positionOnAxisClamped < -halfLength)
            positionOnAxisClamped = -halfLength;
         else if (positionOnAxisClamped > halfLength)
            positionOnAxisClamped = halfLength;

         if (positionOnAxisClamped != positionOnAxis)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.

            double projectionOnAxisXClamped = cylinder3DPosition.getX() + positionOnAxisClamped * cylinder3DAxis.getX();
            double projectionOnAxisYClamped = cylinder3DPosition.getY() + positionOnAxisClamped * cylinder3DAxis.getY();
            double projectionOnAxisZClamped = cylinder3DPosition.getZ() + positionOnAxisClamped * cylinder3DAxis.getZ();

            double toCylinderScale = cylinder3DRadius / distanceFromAxis;
            double closestX = axisToQueryX * toCylinderScale + projectionOnAxisXClamped;
            double closestY = axisToQueryY * toCylinderScale + projectionOnAxisYClamped;
            double closestZ = axisToQueryZ * toCylinderScale + projectionOnAxisZClamped;
            double dX = query.getX() - closestX;
            double dY = query.getY() - closestY;
            double dZ = query.getZ() - closestZ;
            double distance = EuclidCoreTools.norm(dX, dY, dZ);

            closestPointOnSurfaceToPack.set(closestX, closestY, closestZ);

            if (distance < 1.0e-12)
            {
               if (positionOnAxis > 0.0)
                  normalToPack.set(cylinder3DAxis);
               else
                  normalToPack.setAndNegate(cylinder3DAxis);
            }
            else
            {
               normalToPack.set(dX, dY, dZ);
               normalToPack.scale(1.0 / distance);
            }

            return distance;
         }
         else
         { // Closest point is on the cylinder.
            normalToPack.set(axisToQueryX, axisToQueryY, axisToQueryZ);
            normalToPack.scale(1.0 / distanceFromAxis);

            closestPointOnSurfaceToPack.setAndScale(cylinder3DRadius, normalToPack);
            closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);

            return distanceFromAxis - cylinder3DRadius;
         }
      }
   }

   /**
    * Tests whether the {@code query} is located inside an axis-aligned 3D ellipsoid centered at the
    * origin given its radii.
    *
    * @param query            the coordinates of the query. Not modified.
    * @param ellipsoid3DRadii the radii of the ellipsoid. Not modified.
    * @param epsilon          the tolerance to use for this test. A positive value is equivalent to
    *                         growing the size of the ellipsoid, while a negative value is equivalent
    *                         to shrinking it.
    * @return {@code true} if the query is inside or on the ellipsoid's surface, {@code false}
    *         otherwise.
    */
   public static boolean isPoint3DInsideEllipsoid3D(Point3DReadOnly query, Vector3DReadOnly ellipsoid3DRadii, double epsilon)
   {
      double scaledX = query.getX() / (ellipsoid3DRadii.getX() + epsilon);
      double scaledY = query.getY() / (ellipsoid3DRadii.getY() + epsilon);
      double scaledZ = query.getZ() / (ellipsoid3DRadii.getZ() + epsilon);

      return EuclidCoreTools.normSquared(scaledX, scaledY, scaledZ) <= 1.0;
   }

   /**
    * Computes the distance between the {@code query} and an axis-aligned 3D ellipsoid centered at the
    * origin given its radii.
    * <p>
    * The returned distance is signed as follows:
    * <ul>
    * <li>Positive: the query is located outside the ellipsoid.
    * <li>Zero: the query is located on the ellipsoid's surface.
    * <li>Negative: the query is located inside the ellipsoid.
    * </ul>
    * </p>
    *
    * @param query            the coordinates of the query. Not modified.
    * @param ellipsoid3DRadii the radii of the ellipsoid. Not modified.
    * @return the signed distance between the query and the ellipsoid.
    */
   public static double signedDistanceBetweenPoint3DAndEllipsoid3D(Point3DReadOnly query, Vector3DReadOnly ellipsoid3DRadii)
   {
      double xRadius = ellipsoid3DRadii.getX();
      double yRadius = ellipsoid3DRadii.getY();
      double zRadius = ellipsoid3DRadii.getZ();

      double scaleFactor = 1.0 / EuclidCoreTools.norm(query.getX() / xRadius, query.getY() / yRadius, query.getZ() / zRadius);

      return query.distanceFromOrigin() * (1.0 - scaleFactor);
   }

   /**
    * Computes the orthogonal projection of a 3D point on an axis-aligned 3D ellipsoid centered at the
    * origin given its radii.
    * <p>
    * When the query is located inside the ellipsoid, this method returns {@code false} and
    * {@code projectionToPack} is not modified.
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param ellipsoid3DRadii the radii of the ellipsoid. Not modified.
    * @param projectionToPack point in which the projection is stored. Modified.
    * @return whether the projection has succeeded or not.
    */
   public static boolean orthogonalProjectionOntoEllipsoid3D(Point3DReadOnly pointToProject, Vector3DReadOnly ellipsoid3DRadii, Point3DBasics projectionToPack)
   {
      double distance = EuclidEllipsoid3DTools.distancePoint3DEllipsoid3D(ellipsoid3DRadii, pointToProject, projectionToPack);
      return distance > 0.0;
   }

   /**
    * Computes the supporting vertex for an axis-aligned 3D ellipsoid centered at the origin and given
    * its radii.
    * <p>
    * The supporting vertex represents the location on a shape that is the farthest in a given
    * direction.
    * </p>
    *
    * @param supportDirection       the search direction. Not modified.
    * @param ellipsoid3DRadii       the radii of the ellipsoid. Not modified.
    * @param supportingVertexToPack point in which the supporting vertex is stored. Modified.
    */
   public static void supportingVertexEllipsoid3D(Vector3DReadOnly supportDirection, Vector3DReadOnly ellipsoid3DRadii, Point3DBasics supportingVertexToPack)
   {
      double nx = supportDirection.getX();
      double ny = supportDirection.getY();
      double nz = supportDirection.getZ();
      double nLength = EuclidCoreTools.norm(nx, ny, nz);
      nx *= ellipsoid3DRadii.getX() / nLength;
      ny *= ellipsoid3DRadii.getY() / nLength;
      nz *= ellipsoid3DRadii.getZ() / nLength;

      supportingVertexToPack.set(nx, ny, nz);
      supportingVertexToPack.scale(ellipsoid3DRadii.getX(), ellipsoid3DRadii.getY(), ellipsoid3DRadii.getZ());
      supportingVertexToPack.scale(1.0 / EuclidCoreTools.norm(nx, ny, nz));
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given ellipsoid 3D.
    *
    * @param ellipsoid3DPosition    the coordinates of the ellipsoid's center. Not modified.
    * @param ellipsoid3DOrientation the orientation of the ellipsoid. Not modified.
    * @param ellipsoid3DRadii       the radii of the ellipsoid. Not modified.
    * @param boundingBoxToPack      the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxEllipsoid3D(Point3DReadOnly ellipsoid3DPosition, RotationMatrixReadOnly ellipsoid3DOrientation,
                                             Vector3DReadOnly ellipsoid3DRadii, BoundingBox3DBasics boundingBoxToPack)
   {
      double xRange, yRange, zRange;

      if (ellipsoid3DOrientation.isIdentity())
      {
         xRange = ellipsoid3DRadii.getX();
         yRange = ellipsoid3DRadii.getY();
         zRange = ellipsoid3DRadii.getZ();
      }
      else
      {
         double m00 = ellipsoid3DOrientation.getM00() * ellipsoid3DOrientation.getM00();
         double m01 = ellipsoid3DOrientation.getM01() * ellipsoid3DOrientation.getM01();
         double m02 = ellipsoid3DOrientation.getM02() * ellipsoid3DOrientation.getM02();
         double m10 = ellipsoid3DOrientation.getM10() * ellipsoid3DOrientation.getM10();
         double m11 = ellipsoid3DOrientation.getM11() * ellipsoid3DOrientation.getM11();
         double m12 = ellipsoid3DOrientation.getM12() * ellipsoid3DOrientation.getM12();
         double m20 = ellipsoid3DOrientation.getM20() * ellipsoid3DOrientation.getM20();
         double m21 = ellipsoid3DOrientation.getM21() * ellipsoid3DOrientation.getM21();
         double m22 = ellipsoid3DOrientation.getM22() * ellipsoid3DOrientation.getM22();

         double rx = ellipsoid3DRadii.getX() * ellipsoid3DRadii.getX();
         double ry = ellipsoid3DRadii.getY() * ellipsoid3DRadii.getY();
         double rz = ellipsoid3DRadii.getZ() * ellipsoid3DRadii.getZ();

         xRange = EuclidCoreTools.squareRoot(m00 * rx + m01 * ry + m02 * rz);
         yRange = EuclidCoreTools.squareRoot(m10 * rx + m11 * ry + m12 * rz);
         zRange = EuclidCoreTools.squareRoot(m20 * rx + m21 * ry + m22 * rz);
      }

      double maxX = ellipsoid3DPosition.getX() + xRange;
      double maxY = ellipsoid3DPosition.getY() + yRange;
      double maxZ = ellipsoid3DPosition.getZ() + zRange;
      double minX = ellipsoid3DPosition.getX() - xRange;
      double minY = ellipsoid3DPosition.getY() - yRange;
      double minZ = ellipsoid3DPosition.getZ() - zRange;
      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   /**
    * Evaluates the collision between a 3D point and a 3D ellipsoid centered at the origin and given
    * its radii.
    *
    * @param query                       the location of the query. Not modified.
    * @param ellipsoid3DRadii            the radii of the ellipsoid. Not modified.
    * @param closestPointOnSurfaceToPack the closest point located on the surface of the ellipsoid.
    *                                    Modified.
    * @param normalToPack                the surface normal at the closest point. Not modified.
    * @return the signed distance between the query and the ellipsoid. It is negative when the query is
    *         inside, positive otherwise.
    */
   public static double evaluatePoint3DEllipsoid3DCollision(Point3DReadOnly query, Vector3DReadOnly ellipsoid3DRadii, Point3DBasics closestPointOnSurfaceToPack,
                                                            Vector3DBasics normalToPack)
   {
      double distance = EuclidEllipsoid3DTools.distancePoint3DEllipsoid3D(ellipsoid3DRadii, query, closestPointOnSurfaceToPack);
      if (distance != 0.0)
      {
         normalToPack.sub(query, closestPointOnSurfaceToPack);
         normalToPack.scale(1.0 / distance);
      }
      else
      {
         normalToPack.set(closestPointOnSurfaceToPack);
         double rxSquare = ellipsoid3DRadii.getX() * ellipsoid3DRadii.getX();
         double rySquare = ellipsoid3DRadii.getY() * ellipsoid3DRadii.getY();
         double rzSquare = ellipsoid3DRadii.getZ() * ellipsoid3DRadii.getZ();
         normalToPack.scale(1.0 / rxSquare, 1.0 / rySquare, 1.0 / rzSquare);
         normalToPack.normalize();
      }

      return distance;
   }

   /**
    * Computes the length of the slope part of a ramp 3D given its size.
    *
    * @param ramp3DSize the size of the ramp. Not modified.
    * @return the length of the ramp.
    */
   public static double computeRamp3DLength(Vector3DReadOnly ramp3DSize)
   {
      return computeRamp3DLength(ramp3DSize.getX(), ramp3DSize.getZ());
   }

   /**
    * Computes the length of the slope part of a ramp 3D given its size.
    *
    * @param ramp3DSizeX the x-component of the ramp.
    * @param ramp3DSizeZ the z-component of the ramp.
    * @return the length of the ramp.
    */
   public static double computeRamp3DLength(double ramp3DSizeX, double ramp3DSizeZ)
   {
      return EuclidCoreTools.norm(ramp3DSizeX, ramp3DSizeZ);
   }

   /**
    * Computes the incline in radians of a ramp 3D.
    *
    * @param ramp3DSize the size of the ramp. Not modified.
    * @return the slope angle.
    */
   public static double computeRamp3DIncline(Vector3DReadOnly ramp3DSize)
   {
      return computeRamp3DIncline(ramp3DSize.getX(), ramp3DSize.getZ());
   }

   /**
    * Computes the incline in radians of a ramp 3D.
    *
    * @param ramp3DSizeX the x-component of the ramp.
    * @param ramp3DSizeZ the z-component of the ramp.
    * @return the slope angle.
    */
   public static double computeRamp3DIncline(double ramp3DSizeX, double ramp3DSizeZ)
   {
      return EuclidCoreTools.atan(ramp3DSizeZ / ramp3DSizeX);
   }

   /**
    * Computes the centroid of a ramp of the given size.
    *
    * @param ramp3DPose     the pose of the ramp. Not modified.
    * @param size           the ramp's size. Not modified.
    * @param centroidToPack the object used to store the result. Modified.
    */
   public static void computeRamp3DCentroid(Shape3DPoseReadOnly ramp3DPose, Vector3DReadOnly size, Point3DBasics centroidToPack)
   {
      double xLocal = 2.0 / 3.0 * size.getX();
      double yLocal = 0.0;
      double zLocal = 1.0 / 3.0 * size.getZ();

      if (ramp3DPose == null)
      {
         centroidToPack.set(xLocal, yLocal, zLocal);
      }
      else
      {
         double xWorld, yWorld, zWorld;

         if (ramp3DPose.hasRotation())
         {
            RotationMatrixReadOnly ramp3DRotation = ramp3DPose.getRotation();

            xWorld = ramp3DRotation.getM00() * xLocal + ramp3DRotation.getM02() * zLocal;
            yWorld = ramp3DRotation.getM10() * xLocal + ramp3DRotation.getM12() * zLocal;
            zWorld = ramp3DRotation.getM20() * xLocal + ramp3DRotation.getM22() * zLocal;
         }
         else
         {
            xWorld = xLocal;
            yWorld = yLocal;
            zWorld = zLocal;
         }

         xWorld += ramp3DPose.getTranslationX();
         yWorld += ramp3DPose.getTranslationY();
         zWorld += ramp3DPose.getTranslationZ();

         centroidToPack.set(xWorld, yWorld, zWorld);
      }
   }

   /**
    * Tests whether the {@code query} is located inside an axis-aligned 3D ramp starting from the
    * origin.
    *
    * @param query      the coordinates of the query. Not modified.
    * @param ramp3DSize the size of the ramp. Not modified.
    * @param epsilon    the tolerance to use for this test. A positive value is equivalent to growing
    *                   the size of the ramp, while a negative value is equivalent to shrinking it.
    * @return {@code true} if the query is inside or on the ramp's surface, {@code false} otherwise.
    */
   public static boolean isPoint3DInsideRamp3D(Point3DReadOnly query, Vector3DReadOnly ramp3DSize, double epsilon)
   {
      if (query.getZ() < -epsilon)
         return false;

      if (query.getX() > ramp3DSize.getX() + epsilon)
         return false;

      double halfWidth = 0.5 * ramp3DSize.getY() + epsilon;

      if (query.getY() < -halfWidth || query.getY() > halfWidth)
         return false;

      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;

      // Computing the signed distance between the query and the slope face, negative value means the query is below the slope.
      return rampDirectionX * query.getZ() - query.getX() * rampDirectionZ <= epsilon;
   }

   /**
    * Computes the distance between the {@code query} and an axis-aligned 3D ramp starting from the
    * origin.
    * <p>
    * The returned distance is signed as follows:
    * <ul>
    * <li>Positive: the query is located outside the ramp.
    * <li>Zero: the query is located on the ramp's surface.
    * <li>Negative: the query is located inside the ramp.
    * </ul>
    * </p>
    *
    * @param query      the coordinates of the query. Not modified.
    * @param ramp3DSize the size of the ramp. Not modified.
    * @return the signed distance between the query and the ramp.
    */
   public static double signedDistanceBetweenPoint3DAndRamp3D(Point3DReadOnly query, Vector3DReadOnly ramp3DSize)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double halfWidth = 0.5 * ramp3DSize.getY();

      if (query.getZ() < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(query.getX(), 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(query.getY(), -halfWidth, halfWidth);
         double zClosest = 0.0;

         if (xClosest == query.getX() && yClosest == query.getY())
            return -query.getZ();
         else
            return EuclidGeometryTools.distanceBetweenPoint3Ds(xClosest, yClosest, zClosest, query);
      }
      else if (query.getX() > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(query.getX(), query.getZ(), ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(query.getY(), -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(query.getZ(), 0.0, ramp3DSize.getZ());

         if (yClosest == query.getY() && zClosest == query.getZ())
            return query.getX() - ramp3DSize.getX();
         else
            return EuclidGeometryTools.distanceBetweenPoint3Ds(xClosest, yClosest, zClosest, query);
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(query.getX(), query.getZ(), 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(query.getY(), -halfWidth, halfWidth);
         double zClosest = 0.0;

         return EuclidGeometryTools.distanceBetweenPoint3Ds(xClosest, yClosest, zClosest, query);
      }
      else if (Math.abs(query.getY()) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, query.getY());

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(query.getX(), query.getZ(), 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            return Math.abs(query.getY()) - halfWidth;
         }
         else
         { // Query is above the slope
            double dot = query.getX() * rampDirectionX + query.getZ() * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            return EuclidGeometryTools.distanceBetweenPoint3Ds(xClosest, yClosest, zClosest, query);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(query.getX(), query.getZ(), 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         return rampDirectionX * query.getZ() - query.getX() * rampDirectionZ;
      }
      else
      { // Query is inside the ramp
         double distanceToRightFace = -(-halfWidth - query.getY());
         double distanceToLeftFace = halfWidth - query.getY();
         double distanceToRearFace = ramp3DSize.getX() - query.getX();
         double distanceToBottomFace = query.getZ();
         double distanceToSlopeFace = -(rampDirectionX * query.getZ() - query.getX() * rampDirectionZ);

         double minDistance = distanceToRightFace;
         if (minDistance > distanceToLeftFace)
            minDistance = distanceToLeftFace;
         if (minDistance > distanceToRearFace)
            minDistance = distanceToRearFace;
         if (minDistance > distanceToBottomFace)
            minDistance = distanceToBottomFace;
         if (minDistance > distanceToSlopeFace)
            minDistance = distanceToSlopeFace;
         return -minDistance;
      }
   }

   /**
    * Computes the orthogonal projection of a 3D point on an axis-aligned 3D ramp starting from the
    * origin.
    * <p>
    * When the query is located inside the ramp, this method returns {@code false} and
    * {@code projectionToPack} is not modified.
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param ramp3DSize       the size of the ramp. Not modified.
    * @param projectionToPack point in which the projection is stored. Modified.
    * @return whether the projection has succeeded or not.
    */
   public static boolean orthogonalProjectionOntoRamp3D(Point3DReadOnly pointToProject, Vector3DReadOnly ramp3DSize, Point3DBasics projectionToPack)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double xQuery = pointToProject.getX();
      double yQuery = pointToProject.getY();
      double zQuery = pointToProject.getZ();

      double halfWidth = 0.5 * ramp3DSize.getY();

      if (zQuery < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(xQuery, 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(yQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         projectionToPack.set(xClosest, yClosest, zClosest);
         return true;
      }
      else if (xQuery > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(xQuery, zQuery, ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(yQuery, -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(zQuery, 0.0, ramp3DSize.getZ());

         projectionToPack.set(xClosest, yClosest, zClosest);
         return true;
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xQuery, zQuery, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(yQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         projectionToPack.set(xClosest, yClosest, zClosest);
         return true;
      }
      else if (Math.abs(yQuery) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, yQuery);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xQuery, zQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = xQuery;
            double zClosest = zQuery;

            projectionToPack.set(xClosest, yClosest, zClosest);
            return true;
         }
         else
         { // Query is above the slope
            double dot = xQuery * rampDirectionX + zQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            projectionToPack.set(xClosest, yClosest, zClosest);
            return true;
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xQuery, zQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         double dot = xQuery * rampDirectionX + zQuery * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = yQuery;
         double zClosest = dot * rampDirectionZ;

         projectionToPack.set(xClosest, yClosest, zClosest);
         return true;
      }
      else
      { // Query is inside the ramp
         return false;
      }
   }

   /**
    * Computes the supporting vertex for an axis-aligned 3D ramp starting from the origin.
    * <p>
    * The supporting vertex represents the location on a shape that is the farthest in a given
    * direction.
    * </p>
    *
    * @param supportDirection       the search direction. Not modified.
    * @param ramp3DSize             the size of the ramp. Not modified.
    * @param supportingVertexToPack point in which the supporting vertex is stored. Modified.
    */
   public static void supportingVectexRamp3D(Vector3DReadOnly supportDirection, Vector3DReadOnly ramp3DSize, Point3DBasics supportingVertexToPack)
   {
      if (supportDirection.getZ() < 0.0)
      {
         supportingVertexToPack.setX(supportDirection.getX() > 0.0 ? ramp3DSize.getX() : 0.0);
         supportingVertexToPack.setZ(0.0);
      }
      else if (supportDirection.getX() > 0.0)
      {
         supportingVertexToPack.setX(ramp3DSize.getX());
         supportingVertexToPack.setZ(supportDirection.getZ() > 0.0 ? ramp3DSize.getZ() : 0.0);
      }
      else
      { // Look at a the incline of the direction.
         double directionInclineTanArgument = -supportDirection.getX() / supportDirection.getZ();
         double rampInclineTanArgument = ramp3DSize.getZ() / ramp3DSize.getX();

         if (directionInclineTanArgument > rampInclineTanArgument)
         { // Pointing toward the bottom of the ramp
            supportingVertexToPack.setX(0.0);
            supportingVertexToPack.setZ(0.0);
         }
         else
         { // Pointing toward the top of the ramp
            supportingVertexToPack.setX(ramp3DSize.getX());
            supportingVertexToPack.setZ(ramp3DSize.getZ());
         }
      }

      supportingVertexToPack.setY(supportDirection.getY() > 0.0 ? 0.5 * ramp3DSize.getY() : -0.5 * ramp3DSize.getY());
   }

   /**
    * Evaluates the collision between a 3D point and a 3D ramp starting from the origin.
    *
    * @param query                       the location of the query. Not modified.
    * @param ramp3DSize                  the size of the ramp. Not modified.
    * @param closestPointOnSurfaceToPack the closest point located on the surface of the box. Modified.
    * @param normalToPack                the surface normal at the closest point. Not modified.
    * @return the signed distance between the query and the box. It is negative when the query is
    *         inside, positive otherwise.
    */
   public static double evaluatePoint3DRamp3DCollision(Point3DReadOnly query, Vector3DReadOnly ramp3DSize, Point3DBasics closestPointOnSurfaceToPack,
                                                       Vector3DBasics normalToPack)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double xLocalQuery = query.getX();
      double yLocalQuery = query.getY();
      double zLocalQuery = query.getZ();

      double halfWidth = 0.5 * ramp3DSize.getY();

      if (zLocalQuery < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(xLocalQuery, 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);

         if (xClosest == xLocalQuery && yClosest == yLocalQuery)
         {
            normalToPack.setAndNegate(Axis3D.Z);
            return -zLocalQuery;
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (xLocalQuery > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(zLocalQuery, 0.0, ramp3DSize.getZ());

         closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);

         if (yClosest == yLocalQuery && zClosest == zLocalQuery)
         {
            normalToPack.set(Axis3D.X);
            return xLocalQuery - ramp3DSize.getX();
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
         return computeNormalAndDistanceFromClosestPoint(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
      }
      else if (Math.abs(yLocalQuery) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, yLocalQuery);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = xLocalQuery;
            double zClosest = zLocalQuery;

            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);

            if (yLocalQuery >= 0.0)
               normalToPack.set(Axis3D.Y);
            else
               normalToPack.setAndNegate(Axis3D.Y);

            return Math.abs(yLocalQuery) - halfWidth;
         }
         else
         { // Query is above the slope
            double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            return computeNormalAndDistanceFromClosestPoint(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = yLocalQuery;
         double zClosest = dot * rampDirectionZ;

         closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
         normalToPack.set(rampNormalX, 0.0, rampNormalZ);

         return rampDirectionX * zLocalQuery - xLocalQuery * rampDirectionZ;
      }
      else
      { // Query is inside the ramp
         double distanceToRightFace = -(-halfWidth - yLocalQuery);
         double distanceToLeftFace = halfWidth - yLocalQuery;
         double distanceToRearFace = ramp3DSize.getX() - xLocalQuery;
         double distanceToBottomFace = zLocalQuery;
         double distanceToSlopeFace = -(rampDirectionX * zLocalQuery - xLocalQuery * rampDirectionZ);

         if (isFirstValueMinimum(distanceToRightFace, distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the right face
            closestPointOnSurfaceToPack.set(xLocalQuery, -halfWidth, zLocalQuery);
            normalToPack.setAndNegate(Axis3D.Y);
            return -distanceToRightFace;
         }
         else if (isFirstValueMinimum(distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the left face
            closestPointOnSurfaceToPack.set(xLocalQuery, halfWidth, zLocalQuery);
            normalToPack.set(Axis3D.Y);
            return -distanceToLeftFace;
         }
         else if (isFirstValueMinimum(distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the rear face
            closestPointOnSurfaceToPack.set(ramp3DSize.getX(), yLocalQuery, zLocalQuery);
            normalToPack.set(Axis3D.X);
            return -distanceToRearFace;
         }
         else if (distanceToBottomFace <= distanceToSlopeFace)
         { // Query is closer to the bottom face
            closestPointOnSurfaceToPack.set(xLocalQuery, yLocalQuery, 0.0);
            normalToPack.setAndNegate(Axis3D.Z);
            return -distanceToBottomFace;
         }
         else
         { // Query is closer to the slope face
            double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double yClosest = yLocalQuery;
            double zClosest = dot * rampDirectionZ;

            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            normalToPack.set(rampNormalX, 0.0, rampNormalZ);
            return -distanceToSlopeFace;
         }
      }
   }

   private static double computeNormalAndDistanceFromClosestPoint(double xLocalQuery, double yLocalQuery, double zLocalQuery, double xLocalClosest,
                                                                  double yLocalClosest, double zLocalClosest, Vector3DBasics normalToPack)
   {
      double dx = xLocalQuery - xLocalClosest;
      double dy = yLocalQuery - yLocalClosest;
      double dz = zLocalQuery - zLocalClosest;

      double distance = EuclidCoreTools.norm(dx, dy, dz);

      normalToPack.set(dx, dy, dz);
      normalToPack.scale(1.0 / distance);

      return distance;
   }

   /**
    * Tests whether the first argument has the smallest value of all the arguments.
    *
    * @param possibleMin the query.
    * @param value1      the first value to compare the candidate against.
    * @param value2      the second value to compare the candidate against.
    * @param value3      the third value to compare the candidate against.
    * @param value4      the fourth value to compare the candidate against.
    * @return {@code true} if the query has the smallest value, {@code false} otherwise
    */
   public static boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3, double value4)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3 && possibleMin <= value4;
   }

   /**
    * Tests whether the first argument has the smallest value of all the arguments.
    *
    * @param possibleMin the query.
    * @param value1      the first value to compare the candidate against.
    * @param value2      the second value to compare the candidate against.
    * @param value3      the third value to compare the candidate against.
    * @return {@code true} if the query has the smallest value, {@code false} otherwise
    */
   public static boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3;
   }

   /**
    * Tests whether the first argument has the smallest value of all the arguments.
    *
    * @param possibleMin the query.
    * @param value1      the first value to compare the candidate against.
    * @param value2      the second value to compare the candidate against.
    * @return {@code true} if the query has the smallest value, {@code false} otherwise
    */
   public static boolean isFirstValueMinimum(double possibleMin, double value1, double value2)
   {
      return possibleMin <= value1 && possibleMin <= value2;
   }

   /**
    * Tests whether the first argument has the largest value of all the arguments.
    *
    * @param possibleMax the query.
    * @param value1      the first value to compare the candidate against.
    * @param value2      the second value to compare the candidate against.
    * @param value3      the third value to compare the candidate against.
    * @param value4      the fourth value to compare the candidate against.
    * @return {@code true} if the query has the largest value, {@code false} otherwise
    */
   public static boolean isFirstValueMaximum(double possibleMax, double value1, double value2, double value3, double value4)
   {
      return possibleMax >= value1 && possibleMax >= value2 && possibleMax >= value3 && possibleMax >= value4;
   }

   /**
    * Tests whether the first argument has the largest value of all the arguments.
    *
    * @param possibleMax the query.
    * @param value1      the first value to compare the candidate against.
    * @param value2      the second value to compare the candidate against.
    * @param value3      the third value to compare the candidate against.
    * @return {@code true} if the query has the largest value, {@code false} otherwise
    */
   public static boolean isFirstValueMaximum(double possibleMax, double value1, double value2, double value3)
   {
      return possibleMax >= value1 && possibleMax >= value2 && possibleMax >= value3;
   }

   /**
    * Tests whether the first argument has the largest value of all the arguments.
    *
    * @param possibleMax the query.
    * @param value1      the first value to compare the candidate against.
    * @param value2      the second value to compare the candidate against.
    * @return {@code true} if the query has the largest value, {@code false} otherwise
    */
   public static boolean isFirstValueMaximum(double possibleMax, double value1, double value2)
   {
      return possibleMax >= value1 && possibleMax >= value2;
   }

   /**
    * Tests whether the {@code query} is located inside a 3D sphere.
    *
    * @param query            the coordinates of the query. Not modified.
    * @param sphere3DPosition the coordinates of the sphere's center. Not modified.
    * @param sphere3DRadius   the radius of the sphere.
    * @param epsilon          the tolerance to use for this test. A positive value is equivalent to
    *                         growing the size of the sphere, while a negative value is equivalent to
    *                         shrinking it.
    * @return {@code true} if the query is inside or on the sphere's surface, {@code false} otherwise.
    */
   public static boolean isPoint3DInsideSphere3D(Point3DReadOnly query, Point3DReadOnly sphere3DPosition, double sphere3DRadius, double epsilon)
   {
      double radiusWithEpsilon = sphere3DRadius + epsilon;
      return sphere3DPosition.distanceSquared(query) <= radiusWithEpsilon * radiusWithEpsilon;
   }

   /**
    * Computes the distance between the {@code query} and a 3D sphere.
    * <p>
    * The returned distance is signed as follows:
    * <ul>
    * <li>Positive: the query is located outside the sphere.
    * <li>Zero: the query is located on the sphere's surface.
    * <li>Negative: the query is located inside the sphere.
    * </ul>
    * </p>
    *
    * @param query            the coordinates of the query. Not modified.
    * @param sphere3DPosition the coordinates of the sphere's center. Not modified.
    * @param sphere3DRadius   the radius of the sphere.
    * @return the signed distance between the query and the sphere.
    */
   public static double signedDistanceBetweenPoint3DAndSphere3D(Point3DReadOnly query, Point3DReadOnly sphere3DPosition, double sphere3DRadius)
   {
      return sphere3DPosition.distance(query) - sphere3DRadius;
   }

   /**
    * Computes the orthogonal projection of a 3D point on a 3D sphere.
    * <p>
    * When the query is located inside the sphere, this method returns {@code false} and
    * {@code projectionToPack} is not modified.
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param sphere3DPosition the coordinates of the sphere's center. Not modified.
    * @param sphere3DRadius   the radius of the sphere.
    * @param projectionToPack point in which the projection is stored. Modified.
    * @return whether the projection has succeeded or not.
    */
   public static boolean orthogonalProjectionOntoSphere3D(Point3DReadOnly pointToProject, Point3DReadOnly sphere3DPosition, double sphere3DRadius,
                                                          Point3DBasics projectionToPack)
   {
      double distanceSquared = sphere3DPosition.distanceSquared(pointToProject);
      if (distanceSquared <= sphere3DRadius * sphere3DRadius)
         return false;

      projectionToPack.sub(pointToProject, sphere3DPosition);
      projectionToPack.scale(sphere3DRadius / EuclidCoreTools.squareRoot(distanceSquared));
      projectionToPack.add(sphere3DPosition);
      return true;
   }

   /**
    * Computes the supporting vertex for a 3D sphere.
    * <p>
    * The supporting vertex represents the location on a shape that is the farthest in a given
    * direction.
    * </p>
    *
    * @param supportDirection       the search direction. Not modified.
    * @param sphere3DPosition       the coordinates of the sphere's center. Not modified.
    * @param sphere3DRadius         the radius of the sphere.
    * @param supportingVertexToPack point in which the supporting vertex is stored. Modified.
    */
   public static void supportingVertexSphere3D(Vector3DReadOnly supportDirection, Point3DReadOnly sphere3DPosition, double sphere3DRadius,
                                               Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.scaleAdd(sphere3DRadius / supportDirection.length(), supportDirection, sphere3DPosition);
   }

   /**
    * Evaluates the collision between a 3D point and a 3D sphere.
    *
    * @param query                       the location of the query. Not modified.
    * @param sphere3DPosition            the coordinates of the sphere's center. Not modified.
    * @param sphere3DRadius              the radius of the sphere.
    * @param closestPointOnSurfaceToPack the closest point located on the surface of the sphere.
    *                                    Modified.
    * @param normalToPack                the surface normal at the closest point. Not modified.
    * @return the signed distance between the query and the sphere. It is negative when the query is
    *         inside, positive otherwise.
    */
   public static double evaluatePoint3DSphere3DCollision(Point3DReadOnly query, Point3DReadOnly sphere3DPosition, double sphere3DRadius,
                                                         Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      double distance = sphere3DPosition.distance(query);

      if (distance > MIN_DISTANCE_EPSILON)
      {
         closestPointOnSurfaceToPack.sub(query, sphere3DPosition);
         closestPointOnSurfaceToPack.scale(sphere3DRadius / distance);
         normalToPack.sub(query, sphere3DPosition);
         normalToPack.scale(1.0 / distance);
      }
      else
      {
         closestPointOnSurfaceToPack.set(0.0, 0.0, sphere3DRadius);
         normalToPack.set(0.0, 0.0, 1.0);
      }
      closestPointOnSurfaceToPack.add(sphere3DPosition);

      return distance - sphere3DRadius;
   }

   /**
    * Tests whether the {@code query} is located inside a 3D torus.
    *
    * @param query             the coordinates of the query. Not modified.
    * @param torus3DPosition   the coordinates of the torus' center. Not modified.
    * @param torus3DAxis       the axis of revolution of the torus. Not modified.
    * @param torus3DRadius     the radius from the axis to the tube center.
    * @param torus3DTubeRadius the radius of the tube.
    * @param epsilon           the tolerance to use for this test. A positive value is equivalent to
    *                          growing the size of the torus, while a negative value is equivalent to
    *                          shrinking it.
    * @return {@code true} if the query is inside or on the capsule's surface, {@code false} otherwise.
    */
   public static boolean isPoint3DInsideTorus3D(Point3DReadOnly query, Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis, double torus3DRadius,
                                                double torus3DTubeRadius, double epsilon)
   {
      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, torus3DPosition, torus3DAxis);
      double projectionOnAxisX = torus3DPosition.getX() + positionOnAxis * torus3DAxis.getX();
      double projectionOnAxisY = torus3DPosition.getY() + positionOnAxis * torus3DAxis.getY();
      double projectionOnAxisZ = torus3DPosition.getZ() + positionOnAxis * torus3DAxis.getZ();
      double distanceSquaredFromAxis = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ, query);

      double tubeRadiusWithEpsilon = torus3DTubeRadius + epsilon;
      double outerRadius = torus3DRadius + tubeRadiusWithEpsilon;
      double innerRadius = torus3DRadius - tubeRadiusWithEpsilon;

      if (distanceSquaredFromAxis > outerRadius * outerRadius || distanceSquaredFromAxis < innerRadius * innerRadius)
         return false;

      double distanceFromAxis = EuclidCoreTools.squareRoot(distanceSquaredFromAxis);
      return EuclidCoreTools.normSquared(distanceFromAxis - torus3DRadius, positionOnAxis) <= tubeRadiusWithEpsilon * tubeRadiusWithEpsilon;
   }

   /**
    * Computes the distance between the {@code query} and a 3D torus.
    * <p>
    * The returned distance is signed as follows:
    * <ul>
    * <li>Positive: the query is located outside the torus.
    * <li>Zero: the query is located on the torus's surface.
    * <li>Negative: the query is located inside the torus.
    * </ul>
    * </p>
    *
    * @param query             the coordinates of the query. Not modified.
    * @param torus3DPosition   the coordinates of the torus' center. Not modified.
    * @param torus3DAxis       the axis of revolution of the torus. Not modified.
    * @param torus3DRadius     the radius from the axis to the tube center.
    * @param torus3DTubeRadius the radius of the tube.
    * @return the signed distance between the query and the torus.
    */
   public static double signedDistanceBetweenPoint3DAndTorus3D(Point3DReadOnly query, Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis,
                                                               double torus3DRadius, double torus3DTubeRadius)
   {
      double torus3DPositionX = torus3DPosition.getX();
      double torus3DPositionY = torus3DPosition.getY();
      double torus3DPositionZ = torus3DPosition.getZ();
      double torus3DAxisX = torus3DAxis.getX();
      double torus3DAxisY = torus3DAxis.getY();
      double torus3DAxisZ = torus3DAxis.getZ();

      if (!(torus3DAxis instanceof UnitVector3DReadOnly))
      {
         double normInverse = 1.0 / EuclidCoreTools.norm(torus3DAxisX, torus3DAxisY, torus3DAxisZ);

         torus3DAxisX *= normInverse;
         torus3DAxisY *= normInverse;
         torus3DAxisZ *= normInverse;
      }

      double dx = query.getX() - torus3DPositionX;
      double dy = query.getY() - torus3DPositionY;
      double dz = query.getZ() - torus3DPositionZ;
      double positionOnAxis = dx * torus3DAxisX + dy * torus3DAxisY + dz * torus3DAxisZ;

      double projectionOnAxisX = torus3DPosition.getX() + positionOnAxis * torus3DAxisX;
      double projectionOnAxisY = torus3DPosition.getY() + positionOnAxis * torus3DAxisY;
      double projectionOnAxisZ = torus3DPosition.getZ() + positionOnAxis * torus3DAxisZ;
      double distanceSquaredFromAxis = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ, query);

      if (distanceSquaredFromAxis < 1.0e-12)
      {
         return EuclidCoreTools.norm(torus3DRadius, positionOnAxis) - torus3DTubeRadius;
      }
      else
      {
         double distanceFromAxis = EuclidCoreTools.squareRoot(distanceSquaredFromAxis);
         return EuclidCoreTools.norm(distanceFromAxis - torus3DRadius, positionOnAxis) - torus3DTubeRadius;
      }
   }

   /**
    * Computes the orthogonal projection of a 3D point on a 3D torus.
    * <p>
    * When the query is located inside the torus, this method returns {@code false} and
    * {@code projectionToPack} is not modified.
    * </p>
    *
    * @param pointToProject    the point to compute the projection of. Not modified.
    * @param torus3DPosition   the coordinates of the torus' center. Not modified.
    * @param torus3DAxis       the axis of revolution of the torus. Not modified.
    * @param torus3DRadius     the radius from the axis to the tube center.
    * @param torus3DTubeRadius the radius of the tube.
    * @param projectionToPack  point in which the projection is stored. Modified.
    * @return whether the projection has succeeded or not.
    */
   public static boolean orthogonalProjectionOntoTorus3D(Point3DReadOnly pointToProject, Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis,
                                                         double torus3DRadius, double torus3DTubeRadius, Point3DBasics projectionToPack)
   {
      double x = pointToProject.getX() - torus3DPosition.getX();
      double y = pointToProject.getY() - torus3DPosition.getY();
      double z = pointToProject.getZ() - torus3DPosition.getZ();
      double percentageOnAxis = TupleTools.dot(x, y, z, torus3DAxis);

      double xInPlane = x - percentageOnAxis * torus3DAxis.getX();
      double yInPlane = y - percentageOnAxis * torus3DAxis.getY();
      double zInPlane = z - percentageOnAxis * torus3DAxis.getZ();

      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(xInPlane, yInPlane, zInPlane);

      if (distanceSquaredFromAxis < MIN_DISTANCE_EPSILON)
      { // We need to setup a vector that is orthogonal to the torus' axis, then we'll perform the projection along that vector.
         double xNonCollinearToAxis, yNonCollinearToAxis, zNonCollinearToAxis;

         // Purposefully picking a large tolerance to ensure sanity of the cross-product.
         if (Math.abs(torus3DAxis.getY()) > 0.1 || Math.abs(torus3DAxis.getZ()) > 0.1)
         {
            xNonCollinearToAxis = 1.0;
            yNonCollinearToAxis = 0.0;
            zNonCollinearToAxis = 0.0;
         }
         else
         {
            xNonCollinearToAxis = 0.0;
            yNonCollinearToAxis = 1.0;
            zNonCollinearToAxis = 0.0;
         }

         double xOrthogonalToAxis = yNonCollinearToAxis * torus3DAxis.getZ() - zNonCollinearToAxis * torus3DAxis.getY();
         double yOrthogonalToAxis = zNonCollinearToAxis * torus3DAxis.getX() - xNonCollinearToAxis * torus3DAxis.getZ();
         double zOrthogonalToAxis = xNonCollinearToAxis * torus3DAxis.getY() - yNonCollinearToAxis * torus3DAxis.getX();
         double norm = EuclidCoreTools.norm(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis);

         double distanceFromTubeCenter = EuclidCoreTools.norm(percentageOnAxis, torus3DRadius);

         double scale = torus3DTubeRadius / distanceFromTubeCenter;
         double resultDistanceFromAxis = torus3DRadius - torus3DRadius * scale;
         double resultPositionOnAxis = percentageOnAxis * scale;

         projectionToPack.set(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis);
         projectionToPack.scale(resultDistanceFromAxis / norm);
         projectionToPack.scaleAdd(resultPositionOnAxis, torus3DAxis, projectionToPack);
         projectionToPack.add(torus3DPosition);

         return true;
      }
      else
      {
         double distanceFromAxis = EuclidCoreTools.squareRoot(distanceSquaredFromAxis);

         if (EuclidCoreTools.normSquared(distanceFromAxis - torus3DRadius, percentageOnAxis) <= torus3DTubeRadius * torus3DTubeRadius)
            return false;

         double scale = torus3DRadius / distanceFromAxis;

         double xTubeCenter = xInPlane * scale + torus3DPosition.getX();
         double yTubeCenter = yInPlane * scale + torus3DPosition.getY();
         double zTubeCenter = zInPlane * scale + torus3DPosition.getZ();

         double distanceFromTubeCenter = EuclidGeometryTools.distanceBetweenPoint3Ds(xTubeCenter, yTubeCenter, zTubeCenter, pointToProject);

         projectionToPack.set(pointToProject);
         projectionToPack.sub(xTubeCenter, yTubeCenter, zTubeCenter);
         projectionToPack.scale(torus3DTubeRadius / distanceFromTubeCenter);
         projectionToPack.add(xTubeCenter, yTubeCenter, zTubeCenter);

         return true;
      }
   }

   /**
    * Evaluates the collision between a 3D point and a 3D torus.
    *
    * @param query                       the location of the query. Not modified.
    * @param torus3DPosition             the coordinates of the torus' center. Not modified.
    * @param torus3DAxis                 the axis of revolution of the torus. Not modified.
    * @param torus3DRadius               the radius from the axis to the tube center.
    * @param torus3DTubeRadius           the radius of the tube.
    * @param closestPointOnSurfaceToPack the closest point located on the surface of the torus.
    *                                    Modified.
    * @param normalToPack                the surface normal at the closest point. Not modified.
    * @return the signed distance between the query and the torus. It is negative when the query is
    *         inside, positive otherwise.
    */
   public static double evaluatePoint3DTorus3DCollision(Point3DReadOnly query, Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis,
                                                        double torus3DRadius, double torus3DTubeRadius, Point3DBasics closestPointOnSurfaceToPack,
                                                        Vector3DBasics normalToPack)
   {
      double torus3DPositionX = torus3DPosition.getX();
      double torus3DPositionY = torus3DPosition.getY();
      double torus3DPositionZ = torus3DPosition.getZ();
      double torus3DAxisX = torus3DAxis.getX();
      double torus3DAxisY = torus3DAxis.getY();
      double torus3DAxisZ = torus3DAxis.getZ();

      if (!(torus3DAxis instanceof UnitVector3DReadOnly))
      {
         double normInverse = 1.0 / EuclidCoreTools.norm(torus3DAxisX, torus3DAxisY, torus3DAxisZ);

         torus3DAxisX *= normInverse;
         torus3DAxisY *= normInverse;
         torus3DAxisZ *= normInverse;
      }

      double x = query.getX() - torus3DPositionX;
      double y = query.getY() - torus3DPositionY;
      double z = query.getZ() - torus3DPositionZ;
      double percentageOnAxis = x * torus3DAxisX + y * torus3DAxisY + z * torus3DAxisZ;

      double xInPlane = x - percentageOnAxis * torus3DAxisX;
      double yInPlane = y - percentageOnAxis * torus3DAxisY;
      double zInPlane = z - percentageOnAxis * torus3DAxisZ;

      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(xInPlane, yInPlane, zInPlane);

      if (distanceSquaredFromAxis < MIN_DISTANCE_EPSILON)
      { // We need to setup a vector that is orthogonal to the torus' axis, then we'll perform the projection along that vector.
        // Purposefully picking a large tolerance to ensure sanity of the cross-product.
         if (Math.abs(torus3DAxis.getY()) > 0.1 || Math.abs(torus3DAxis.getZ()) > 0.1)
            normalToPack.set(1.0, 0.0, 0.0);
         else
            normalToPack.set(0.0, 1.0, 0.0);

         normalToPack.cross(torus3DAxis);
         normalToPack.normalize();

         double distanceFromTubeCenter = EuclidCoreTools.norm(percentageOnAxis, torus3DRadius);

         double scale = torus3DTubeRadius / distanceFromTubeCenter;
         double resultDistanceFromAxis = torus3DRadius - torus3DRadius * scale;
         double resultPositionOnAxis = percentageOnAxis * scale;

         closestPointOnSurfaceToPack.set(normalToPack);
         closestPointOnSurfaceToPack.scale(resultDistanceFromAxis);
         closestPointOnSurfaceToPack.set(resultPositionOnAxis * torus3DAxisX + closestPointOnSurfaceToPack.getX(),
                                         resultPositionOnAxis * torus3DAxisY + closestPointOnSurfaceToPack.getY(),
                                         resultPositionOnAxis * torus3DAxisZ + closestPointOnSurfaceToPack.getZ());
         closestPointOnSurfaceToPack.add(torus3DPositionX, torus3DPositionY, torus3DPositionZ);

         normalToPack.scale(-torus3DRadius);
         normalToPack.set(percentageOnAxis * torus3DAxisX + normalToPack.getX(),
                          percentageOnAxis * torus3DAxisY + normalToPack.getY(),
                          percentageOnAxis * torus3DAxisZ + normalToPack.getZ());
         normalToPack.scale(1.0 / distanceFromTubeCenter);

         return distanceFromTubeCenter - torus3DTubeRadius;
      }
      else
      {
         double distanceFromAxis = EuclidCoreTools.squareRoot(distanceSquaredFromAxis);

         double scale = torus3DRadius / distanceFromAxis;

         double xTubeCenter = xInPlane * scale + torus3DPositionX;
         double yTubeCenter = yInPlane * scale + torus3DPositionY;
         double zTubeCenter = zInPlane * scale + torus3DPositionZ;

         double distanceSquaredFromTubeCenter = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(xTubeCenter, yTubeCenter, zTubeCenter, query);

         if (distanceSquaredFromTubeCenter < MIN_DISTANCE_EPSILON)
         { // Second edge-case: the query is on the tube center.
           // However, here we know that torus3DAxis is orthogonal to the local tube axis, let just pick that one.
            normalToPack.set(torus3DAxisX, torus3DAxisY, torus3DAxisZ);
            closestPointOnSurfaceToPack.set(xTubeCenter, yTubeCenter, zTubeCenter);
            closestPointOnSurfaceToPack.scaleAdd(torus3DTubeRadius, normalToPack, closestPointOnSurfaceToPack);
            return -torus3DTubeRadius;
         }
         else
         {
            double distanceFromTubeCenter = EuclidCoreTools.squareRoot(distanceSquaredFromTubeCenter);
            normalToPack.set(query);
            normalToPack.sub(xTubeCenter, yTubeCenter, zTubeCenter);
            normalToPack.scale(1.0 / distanceFromTubeCenter);

            closestPointOnSurfaceToPack.setAndScale(torus3DTubeRadius, normalToPack);
            closestPointOnSurfaceToPack.add(xTubeCenter, yTubeCenter, zTubeCenter);

            return distanceFromTubeCenter - torus3DTubeRadius;
         }
      }
   }

   /**
    * Computes the supporting vertex for a circle positioned in the 3D space.
    * <p>
    * The supporting vertex represents the location on a shape that is the farthest in a given
    * direction.
    * </p>
    * 
    * @param supportDirection       the search direction. Not modified.
    * @param circle3DPosition       the location of the circle's center. Not modified.
    * @param circle3DAxis           the axis of revolution of the circle. Not modified.
    * @param circle3DRadius         the radius of the circle.
    * @param supportingVertexToPack point in which the supporting vertex is stored. Modified.
    */
   public static void supportingVertexCircle3D(Vector3DReadOnly supportDirection, Point3DReadOnly circle3DPosition, Vector3DReadOnly circle3DAxis,
                                               double circle3DRadius, Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.set(supportDirection);
      double dot = supportDirection.dot(circle3DAxis);
      supportingVertexToPack.setAndScale(dot / circle3DAxis.lengthSquared(), circle3DAxis);
      supportingVertexToPack.sub(supportDirection, supportingVertexToPack);
      double distanceSquaredFromCenter = supportingVertexToPack.distanceFromOriginSquared();

      if (distanceSquaredFromCenter < MIN_DISTANCE_EPSILON)
      { // We need to setup a vector that is orthogonal to the circle's axis, then we'll perform the projection along that vector.
         double xNonCollinearToAxis, yNonCollinearToAxis, zNonCollinearToAxis;

         // Purposefully picking a large tolerance to ensure sanity of the cross-product.
         if (Math.abs(circle3DAxis.getY()) > 0.1 || Math.abs(circle3DAxis.getZ()) > 0.1)
         {
            xNonCollinearToAxis = 1.0;
            yNonCollinearToAxis = 0.0;
            zNonCollinearToAxis = 0.0;
         }
         else
         {
            xNonCollinearToAxis = 0.0;
            yNonCollinearToAxis = 1.0;
            zNonCollinearToAxis = 0.0;
         }

         double xOrthogonalToAxis = yNonCollinearToAxis * circle3DAxis.getZ() - zNonCollinearToAxis * circle3DAxis.getY();
         double yOrthogonalToAxis = zNonCollinearToAxis * circle3DAxis.getX() - xNonCollinearToAxis * circle3DAxis.getZ();
         double zOrthogonalToAxis = xNonCollinearToAxis * circle3DAxis.getY() - yNonCollinearToAxis * circle3DAxis.getX();

         supportingVertexToPack.set(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis);
         supportingVertexToPack.scale(circle3DRadius / EuclidCoreTools.norm(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis));
      }
      else
      {
         supportingVertexToPack.scale(circle3DRadius / EuclidCoreTools.squareRoot(distanceSquaredFromCenter));
      }

      supportingVertexToPack.add(circle3DPosition);
   }

   /**
    * Computes the supporting vertex for a torus while restricting the solution to lie on the inner
    * part of the torus only.
    * 
    * @param supportDirection       the search direction. Not modified.
    * @param torus3DPosition        the coordinates of the torus' center. Not modified.
    * @param torus3DAxis            the axis of revolution of the torus. Not modified.
    * @param torus3DRadius          the radius from the axis to the tube center.
    * @param torus3DTubeRadius      the radius of the tube.
    * @param supportingVertexToPack point in which the supporting vertex is stored. Modified.
    */
   public static void innerSupportingVertexTorus3D(Vector3DReadOnly supportDirection, Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis,
                                                   double torus3DRadius, double torus3DTubeRadius, Point3DBasics supportingVertexToPack)
   {
      double torus3DAxisX = torus3DAxis.getX();
      double torus3DAxisY = torus3DAxis.getY();
      double torus3DAxisZ = torus3DAxis.getZ();

      // The first part is searching for the support vertex in the opposite direction of supportDirection.
      // This re-using the supporting vertex computation for a circle 3D, when the circle represents the center of the torus' tube.
      supportingVertexToPack.setAndNegate(supportDirection);
      double dot = TupleTools.dot(supportingVertexToPack, torus3DAxis);
      supportingVertexToPack.setAndScale(dot / torus3DAxis.lengthSquared(), torus3DAxis);
      supportingVertexToPack.add(supportDirection);
      supportingVertexToPack.negate();
      double distanceSquaredFromCenter = supportingVertexToPack.distanceFromOriginSquared();

      if (distanceSquaredFromCenter < MIN_DISTANCE_EPSILON)
      { // We need to setup a vector that is orthogonal to the circle's axis, then we'll perform the projection along that vector.
         double xNonCollinearToAxis, yNonCollinearToAxis, zNonCollinearToAxis;

         // Purposefully picking a large tolerance to ensure sanity of the cross-product.
         if (Math.abs(torus3DAxisY) > 0.1 || Math.abs(torus3DAxisZ) > 0.1)
         {
            xNonCollinearToAxis = 1.0;
            yNonCollinearToAxis = 0.0;
            zNonCollinearToAxis = 0.0;
         }
         else
         {
            xNonCollinearToAxis = 0.0;
            yNonCollinearToAxis = 1.0;
            zNonCollinearToAxis = 0.0;
         }

         double xOrthogonalToAxis = yNonCollinearToAxis * torus3DAxisZ - zNonCollinearToAxis * torus3DAxisY;
         double yOrthogonalToAxis = zNonCollinearToAxis * torus3DAxisX - xNonCollinearToAxis * torus3DAxisZ;
         double zOrthogonalToAxis = xNonCollinearToAxis * torus3DAxisY - yNonCollinearToAxis * torus3DAxisX;

         supportingVertexToPack.set(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis);
         supportingVertexToPack.scale(torus3DRadius / EuclidCoreTools.norm(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis));
      }
      else
      {
         supportingVertexToPack.scale(torus3DRadius / EuclidCoreTools.squareRoot(distanceSquaredFromCenter));
      }

      double tubeCenterX = supportingVertexToPack.getX();
      double tubeCenterY = supportingVertexToPack.getY();
      double tubeCenterZ = supportingVertexToPack.getZ();

      // Now we have the position of the tube section where we need to find the supporting vertex.
      double toTubeCenterX = tubeCenterX / torus3DRadius;
      double toTubeCenterY = tubeCenterY / torus3DRadius;
      double toTubeCenterZ = tubeCenterZ / torus3DRadius;

      double tubeLocalAxisX = torus3DAxisY * toTubeCenterZ - torus3DAxisZ * toTubeCenterY;
      double tubeLocalAxisY = torus3DAxisZ * toTubeCenterX - torus3DAxisX * toTubeCenterZ;
      double tubeLocalAxisZ = torus3DAxisX * toTubeCenterY - torus3DAxisY * toTubeCenterX;
      double tubeLocalAxisNormSqured = EuclidCoreTools.normSquared(tubeLocalAxisX, tubeLocalAxisY, tubeLocalAxisZ);

      dot = TupleTools.dot(tubeLocalAxisX, tubeLocalAxisY, tubeLocalAxisZ, supportDirection) / tubeLocalAxisNormSqured;
      supportingVertexToPack.set(supportDirection.getX() - dot * tubeLocalAxisX,
                                 supportDirection.getY() - dot * tubeLocalAxisY,
                                 supportDirection.getZ() - dot * tubeLocalAxisZ);
      distanceSquaredFromCenter = supportingVertexToPack.distanceFromOriginSquared();

      if (distanceSquaredFromCenter < MIN_DISTANCE_EPSILON)
      { // We'll compute the vertex that is the closest to the torus axis.
         supportingVertexToPack.set(tubeLocalAxisX, tubeLocalAxisY, tubeLocalAxisZ);
         supportingVertexToPack.scale(-torus3DTubeRadius / EuclidCoreTools.squareRoot(tubeLocalAxisNormSqured));
      }
      else
      {
         supportingVertexToPack.scale(torus3DTubeRadius / EuclidCoreTools.squareRoot(distanceSquaredFromCenter));
      }

      supportingVertexToPack.add(tubeCenterX, tubeCenterY, tubeCenterZ);
      supportingVertexToPack.add(torus3DPosition);
   }

   public static double boxVolume(double sizeX, double sizeY, double sizeZ)
   {
      return sizeX * sizeY * sizeZ;
   }

   public static double capsuleVolume(double radius, double length)
   {
      return Math.PI * radius * radius * (4.0 / 3.0 * radius + length);
   }

   /**
    * Computes the volume of a cone defined by its height and base radius.
    *
    * @param height the height of the cone.
    * @param radius the radius of the cone's base.
    * @return the volume of the cone.
    */
   public static double coneVolume(double height, double radius)
   {
      return Math.PI * radius * radius * height / 3.0;
   }

   /**
    * Computes the volume of a cylinder defined by its length and radius.
    *
    * @param length the length of the cylinder.
    * @param radius the radius of the cylinder.
    * @return the volume of the cylinder.
    */
   public static double cylinderVolume(double length, double radius)
   {
      return Math.PI * radius * radius * length;
   }

   public static double ellipsoidVolume(double radiusX, double radiusY, double radiusZ)
   {
      return 4.0 / 3.0 * Math.PI * radiusX * radiusY * radiusZ;
   }

   /**
    * Computes the volume of a regular icosahedron defined by its edge length.
    *
    * @param edgeLength the length of each edge of the icosahedron.
    * @return the volume of the icosahedron.
    */
   public static double icosahedronVolume(double edgeLength)
   {
      return edgeLength * edgeLength * edgeLength * (5.0 * (3.0 + EuclidCoreTools.squareRoot(5.0))) / 12.0;
   }

   /**
    * Computes the volume of a pyramid defined by its height and the size of its base.
    *
    * @param height     the height of the pyramid.
    * @param baseLength the length of the pyramid's base.
    * @param baseWidth  the width of the pyramid's base.
    * @return the volume of the pyramid.
    */
   public static double pyramidVolume(double height, double baseLength, double baseWidth)
   {
      return baseLength * baseWidth * height / 3.0;
   }

   public static double rampVolume(double sizeX, double sizeY, double sizeZ)
   {
      return 0.5 * boxVolume(sizeX, sizeY, sizeZ);
   }

   public static double sphereVolume(double radius)
   {
      return ellipsoidVolume(radius, radius, radius);
   }

   /**
    * Computes the volume of a tetrahedron defined by its vertices.
    *
    * @param a the coordinates of the first vertex of the tetrahedron. Not modified.
    * @param b the coordinates of the second vertex of the tetrahedron. Not modified.
    * @param c the coordinates of the third vertex of the tetrahedron. Not modified.
    * @param d the coordinates of the fourth vertex of the tetrahedron. Not modified.
    * @return the volume of the tetrahedron.
    */
   public static double tetrahedronVolume(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c, Point3DReadOnly d)
   {
      // From: http://mathworld.wolfram.com/Tetrahedron.html
      // V = 1/3 * | v1 . (v2 x v3)|
      // where the vectors v1, v2, and v3 are defined as follows:
      // v1 = b - a
      // v2 = c - a
      // v3 = d - a
      double x = (c.getY() - a.getY()) * (d.getZ() - a.getZ()) - (c.getZ() - a.getZ()) * (d.getY() - a.getY());
      double y = (c.getZ() - a.getZ()) * (d.getX() - a.getX()) - (c.getX() - a.getX()) * (d.getZ() - a.getZ());
      double z = (c.getX() - a.getX()) * (d.getY() - a.getY()) - (c.getY() - a.getY()) * (d.getX() - a.getX());
      return Math.abs(x * (b.getX() - a.getX()) + y * (b.getY() - a.getY()) + z * (b.getZ() - a.getZ())) / 6.0;
   }

   public static double torusVolume(double radius, double tubeRadius)
   {
      return 2.0 * Math.PI * Math.PI * radius * tubeRadius * tubeRadius;
   }

   /**
    * Computes the edge length of a regular icosahedron given the radius of its circumscribed sphere.
    *
    * @param radius the radius of the circumscribed sphere.
    * @return the edge length of the regular icosahedron.
    */
   public static double icosahedronEdgeLength(double radius)
   {
      return radius / EuclidCoreTools.sin(0.4 * Math.PI);
   }

   /**
    * Computes the radius of the circumscribed sphere of a regular icosahedron given its edge length.
    *
    * @param edgeLength the edge length.
    * @return the radius of the circumscribed sphere.
    */
   public static double icosahedronRadius(double edgeLength)
   {
      return edgeLength * EuclidCoreTools.sin(0.4 * Math.PI);
   }

   /**
    * Variation of {@link Point3DReadOnly#geometricallyEquals(Point3DReadOnly, double)} allowing to
    * compare the two points by independently measuring the error along and orthogonal to a given
    * normal vector.
    *
    * @param expected          the expected value. Not modified.
    * @param actual            the actual value. Not modified.
    * @param normal            the normal vector used to measure the normal error and the tangential
    *                          error. Not modified.
    * @param normalEpsilon     the tolerance for the error along the normal vector.
    * @param tangentialEpsilon the tolerance for the error orthogonal to the normal vector.
    * @return {@code true} if the two points are considered to represent the same geometry,
    *         {@code false} otherwise.
    */
   public static boolean geometricallyEquals(Point3DReadOnly expected, Point3DReadOnly actual, Vector3DReadOnly normal, double normalEpsilon,
                                             double tangentialEpsilon)
   {
      double normalX = normal.getX();
      double normalY = normal.getY();
      double normalZ = normal.getZ();
      return geometricallyEquals(expected, actual, normalX, normalY, normalZ, normalEpsilon, tangentialEpsilon);
   }

   /**
    * Variation of {@link Point3DReadOnly#geometricallyEquals(Point3DReadOnly, double)} allowing to
    * compare the two points by independently measuring the error along and orthogonal to a given
    * normal vector.
    *
    * @param expected          the expected value. Not modified.
    * @param actual            the actual value. Not modified.
    * @param normalX           the x-component of the normal vector used to measure the normal error
    *                          and the tangential error. Not modified.
    * @param normalY           the y-component of the normal vector used to measure the normal error
    *                          and the tangential error. Not modified.
    * @param normalZ           the z-component of the normal vector used to measure the normal error
    *                          and the tangential error. Not modified.
    * @param normalEpsilon     the tolerance for the error along the normal vector.
    * @param tangentialEpsilon the tolerance for the error orthogonal to the normal vector.
    * @return {@code true} if the two points are considered to represent the same geometry,
    *         {@code false} otherwise.
    */
   public static boolean geometricallyEquals(Point3DReadOnly expected, Point3DReadOnly actual, double normalX, double normalY, double normalZ,
                                             double normalEpsilon, double tangentialEpsilon)
   {
      double normalLengthSquared = EuclidCoreTools.normSquared(normalX, normalY, normalZ);

      if (!EuclidCoreTools.epsilonEquals(1.0, normalLengthSquared, 1.0e-12))
      {
         double normalLengthInverse = 1.0 / EuclidCoreTools.fastSquareRoot(normalLengthSquared);
         normalX *= normalLengthInverse;
         normalY *= normalLengthInverse;
         normalZ *= normalLengthInverse;
      }

      double errorX = expected.getX() - actual.getX();
      double errorY = expected.getY() - actual.getY();
      double errorZ = expected.getZ() - actual.getZ();

      double errorNormal = errorX * normalX + errorY * normalY + errorZ * normalZ;

      if (!EuclidCoreTools.isZero(errorNormal, normalEpsilon))
         return false;

      double errorNormalX = errorNormal * normalX;
      double errorNormalY = errorNormal * normalY;
      double errorNormalZ = errorNormal * normalZ;

      double errorTangentialX = errorX - errorNormalX;
      double errorTangentialY = errorY - errorNormalY;
      double errorTangentialZ = errorZ - errorNormalZ;
      double errorTangential = EuclidCoreTools.norm(errorTangentialX, errorTangentialY, errorTangentialZ);

      return EuclidCoreTools.isZero(errorTangential, tangentialEpsilon);
   }
}
