package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Box3DReadOnly extends Shape3DReadOnly
{
   Vector3DReadOnly getSize();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN() || getSize().containsNaN();
   }

   @Override
   default boolean checkIfInside(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(pointToCheck, queryInLocal);
      boolean isInside = EuclidShapeTools.evaluatePoint3DWithBox3D(queryInLocal, closestPointOnSurfaceToPack, normalAtClosestPointToPack, getSize()) <= 0.0;

      if (closestPointOnSurfaceToPack != null)
         transformToWorld(closestPointOnSurfaceToPack);

      if (normalAtClosestPointToPack != null)
         transformToWorld(normalAtClosestPointToPack);

      return isInside;
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(point, queryInLocal);
      return EuclidShapeTools.signedDistanceBetweenPoint3DAndBox3D(queryInLocal, getSize());
   }

   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(query, queryInLocal);
      return EuclidShapeTools.isPoint3DInsideBox3D(queryInLocal, getSize(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      Point3DBasics pointInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(pointToProject, pointInLocal);

      boolean isInside = EuclidShapeTools.orthogonalProjection(pointInLocal, projectionToPack, getSize());

      if (isInside)
      {
         if (projectionToPack != pointToProject)
            projectionToPack.set(pointToProject);
      }
      else
      {
         transformToWorld(projectionToPack);
      }

      return !isInside;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line the line expressed in world coordinates that may intersect this box. Not modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    */
   default int intersectionWith(Line3DReadOnly line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine a point expressed in world located on the infinitely long line. Not modified.
    * @param lineDirection the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    */
   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {
      double maxX = 0.5 * getSizeX();
      double maxY = 0.5 * getSizeY();
      double maxZ = 0.5 * getSizeZ();
      double minX = -maxX;
      double minY = -maxY;
      double minZ = -maxZ;

      Point3DBasics pointInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      Vector3DBasics vectorInLocal = getIntermediateVariableSupplier().getVector3D(0);

      getPose().inverseTransform(pointOnLine, pointInLocal);
      getPose().inverseTransform(lineDirection, vectorInLocal);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ, pointInLocal, vectorInLocal,
                                                                                                firstIntersectionToPack, secondIntersectionToPack);
      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack);
      return numberOfIntersections;
   }

   /**
    * Convenience method that packs the center of the box.
    * <p>
    * This method is equivalent to {@link #getPosition(Tuple3DBasics)}.
    * </p>
    *
    * @param centerToPack the point in which the coordinates of the center of this box are stored.
    *           Modified.
    */
   default void getCenter(Point3DBasics centerToPack)
   {
      getPosition(centerToPack);
   }

   /**
    * Gets this box size along the x-axis, i.e. its length.
    *
    * @return this box size along the x-axis.
    */
   default double getSizeX()
   {
      return getSize().getX();
   }

   /**
    * Gets this box size along the y-axis, i.e. its width.
    *
    * @return this box size along the y-axis.
    */
   default double getSizeY()
   {
      return getSize().getY();
   }

   /**
    * Gets this box size along the z-axis, i.e. its height.
    *
    * @return this box size along the z-axis.
    */
   default double getSizeZ()
   {
      return getSize().getZ();
   }

   /**
    * Computes and packs the bounding box 3D that fully contains this box 3D.
    * <p>
    * This method does not take the size of this box to convert it into a bounding box, it actually
    * considers the pose of this box 3D such that if it has a non-zero orientation the bounding box
    * will be bigger than this box.
    * </p>
    *
    * @param boundingBoxToPack the bounding box to pack. Modified.
    */
   default void getBoundingBox3D(BoundingBox3D boundingBoxToPack)
   {
      boundingBoxToPack.setToNaN();
      Point3DBasics vertex = getIntermediateVariableSupplier().getPoint3D(0);

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
      {
         getVertex(vertexIndex, vertex);
         boundingBoxToPack.updateToIncludePoint(vertex);
      }
   }

   /**
    * Gets the 8 vertices, expressed in world, of this box as an array.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @return an array of 8 {@code Point3D} with this box vertices.
    */
   default Point3D[] getVertices()
   {
      Point3D[] vertices = new Point3D[8];
      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new Point3D());
      return vertices;
   }

   /**
    * Pack the coordinates in world of the 8 vertices of this box in the given array.
    *
    * @param verticesToPack the array in which the coordinates are stored. Modified.
    * @throws IllegalArgumentException if the length of the given array is different than 8.
    * @throws NullPointerException if any of the 8 first elements of the given array is {@code null}.
    */
   default void getVertices(Point3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 8)
         throw new IllegalArgumentException("Array is too small, has to be at least 8 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   /**
    * Packs the world coordinates of one of this box vertices.
    *
    * @param vertexIndex the index in [0, 7] of the vertex to pack.
    * @param vertexToPack point in which the coordinates of the vertex are stored. Modified.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 7].
    */
   default void getVertex(int vertexIndex, Point3DBasics vertexToPack)
   {
      if (vertexIndex < 0 || vertexIndex >= 8)
         throw new IndexOutOfBoundsException("The vertex index has to be in [0, 7], was: " + vertexIndex);

      vertexToPack.setX((vertexIndex & 1) == 0 ? getSizeX() : -getSizeX());
      vertexToPack.setY((vertexIndex & 2) == 0 ? getSizeY() : -getSizeY());
      vertexToPack.setZ((vertexIndex & 4) == 0 ? getSizeZ() : -getSizeZ());
      vertexToPack.scale(0.5);
      transformToWorld(vertexToPack);
   }

   /**
    * Tests separately and on a per component basis if the pose and the size of this box and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other box which pose and size is to be compared against this box pose and size.
    *           Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two boxes are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Box3DReadOnly other, double epsilon)
   {
      return getSize().epsilonEquals(other.getSize(), epsilon) && getOrientation().epsilonEquals(other.getOrientation(), epsilon)
            && getPosition().epsilonEquals(other.getPosition(), epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two boxes are geometrically similar.
    * <p>
    * This method accounts for the multiple combinations of sizes and rotations that generate identical
    * boxes. For instance, two boxes that are identical but one is flipped by 180 degrees are
    * considered geometrically equal.
    * </p>
    *
    * @param other the box to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two boxes represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Box3DReadOnly other, double epsilon)
   {
      if (!getPosition().geometricallyEquals(getPosition(), epsilon))
         return false;

      Vector3DBasics otherSize = getIntermediateVariableSupplier().getVector3D(0);
      other.getPose().transform(other.getSize(), otherSize);
      transformToLocal(otherSize);

      return getSize().geometricallyEquals(otherSize, epsilon);
   }
}
