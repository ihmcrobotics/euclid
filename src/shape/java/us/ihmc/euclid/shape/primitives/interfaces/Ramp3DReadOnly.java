package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Ramp3DReadOnly extends Shape3DReadOnly
{
   Vector3DReadOnly getSize();

   Shape3DPoseReadOnly getPose();

   /**
    * Gets the read-only reference to the orientation of this shape.
    *
    * @return the orientation of this shape.
    */
   default RotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the read-only reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   default Point3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   IntermediateVariableSupplier getIntermediateVariableSupplier();

   /**
    * Gets the length of this ramp's slope part.
    * <p>
    * Note that this is different than {@link #getSizeX()}. The returned value is equal to:
    * &radic;(this.length<sup>2</sup> + this.height<sup>2</sup>)
    * </p>
    *
    * @return the length of the slope.
    */
   default double getRampLength()
   {
      return EuclidShapeTools.computeRamp3DLength(getSize());
   }

   /**
    * Gets the angle formed by the slope and the bottom face.
    * <p>
    * The angle is positive and in [0, <i>pi</i>].
    * </p>
    *
    * @return the slope angle.
    */
   default double getRampIncline()
   {
      return EuclidShapeTools.computeRamp3DIncline(getSize());
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getPose().containsNaN() || getSize().containsNaN();
   }

   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics pointToCheckInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToCheck, pointToCheckInLocal);

      double distance = EuclidShapeTools.evaluatePoint3DRamp3DCollision(pointToCheckInLocal, getSize(), closestPointOnSurfaceToPack,
                                                                        normalAtClosestPointToPack);

      transformToWorld(closestPointOnSurfaceToPack);
      transformToWorld(normalAtClosestPointToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointToCheckInLocal);

      return distance <= 0.0;
   }

   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      Vector3DBasics supportDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();
      getPose().inverseTransform(supportDirection, supportDirectionInLocal);

      EuclidShapeTools.supportingVectexRamp3D(supportDirectionInLocal, getSize(), supportingVertexToPack);

      transformToWorld(supportingVertexToPack);

      getIntermediateVariableSupplier().releaseVector3D(supportDirectionInLocal);

      return true;
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics pointInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(point, pointInLocal);

      double signedDistance = EuclidShapeTools.signedDistanceBetweenPoint3DAndRamp3D(pointInLocal, getSize());

      getIntermediateVariableSupplier().releasePoint3D(pointInLocal);

      return signedDistance;
   }

   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(query, queryInLocal);

      boolean isInside = EuclidShapeTools.isPoint3DInsideRamp3D(queryInLocal, getSize(), epsilon);

      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      return isInside;
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      Point3DBasics pointToProjectInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToProject, pointToProjectInLocal);

      boolean hasBeenProjected = EuclidShapeTools.orthogonalProjectionOntoRamp3D(pointToProjectInLocal, getSize(), projectionToPack);

      if (hasBeenProjected)
         transformToWorld(projectionToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointToProjectInLocal);

      return hasBeenProjected;
   }

   /**
    * Gets the size of this ramp along the x-axis.
    *
    * @return this ramp's length.
    */
   default double getSizeX()
   {
      return getSize().getX();
   }

   /**
    * Gets the size of this ramp along the y-axis.
    *
    * @return this ramp's width.
    */
   default double getSizeY()
   {
      return getSize().getY();
   }

   /**
    * Gets the size of this ramp along the z-axis.
    *
    * @return this ramp's height.
    */
   default double getSizeZ()
   {
      return getSize().getZ();
   }

   default Vector3D getRampSurfaceNormal()
   {
      Vector3D surfaceNormal = new Vector3D();
      getRampSurfaceNormal(surfaceNormal);
      return surfaceNormal;
   }

   /**
    * Computes and packs the surface normal of the slope face of this ramp.
    *
    * @param surfaceNormalToPack the surface normal of the slope. Modified.
    */
   default void getRampSurfaceNormal(Vector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.set(-getSizeZ() / getRampLength(), 0.0, getSizeX() / getRampLength());
      transformToWorld(surfaceNormalToPack);
   }

   default Point3DBasics[] getVertices()
   {
      Point3D[] vertices = new Point3D[6];
      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new Point3D());
      return vertices;
   }

   default void getVertices(Point3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 6)
         throw new IllegalArgumentException("Array is too small, has to be at least 6 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   default Point3DBasics getVertex(int vertexIndex)
   {
      Point3D vertex = new Point3D();
      getVertex(vertexIndex, vertex);
      return vertex;
   }

   default void getVertex(int vertexIndex, Point3DBasics vertexToPack)
   {
      if (vertexIndex < 0 || vertexIndex >= 6)
         throw new IndexOutOfBoundsException("The vertex index has to be in [0, 5], was: " + vertexIndex);

      vertexToPack.setX((vertexIndex & 2) == 0 ? getSizeX() : 0.0);
      vertexToPack.setY((vertexIndex & 1) == 0 ? 0.5 * getSizeY() : -0.5 * getSizeY());
      vertexToPack.setZ((vertexIndex & 4) == 0 ? 0.0 : getSizeZ());
      transformToWorld(vertexToPack);
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.setToNaN();
      Point3DBasics vertex = getIntermediateVariableSupplier().requestPoint3D();

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
      {
         getVertex(vertexIndex, vertex);
         boundingBoxToPack.updateToIncludePoint(vertex);
      }

      getIntermediateVariableSupplier().releasePoint3D(vertex);
   }

   @Override
   default boolean isConvex()
   {
      return true;
   }

   /**
    * Tests separately and on a per component basis if the pose and the size of this ramp and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other ramp which pose and size is to be compared against this ramp pose and
    *           size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ramps are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Ramp3DReadOnly other, double epsilon)
   {
      return getSize().epsilonEquals(other.getSize(), epsilon) && getPosition().epsilonEquals(other.getPosition(), epsilon)
            && getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ramps are geometrically similar,
    * i.e. the difference between their size are less than or equal to {@code epsilon} and their poses
    * are geometrically similar given {@code epsilon}.
    *
    * @param other the ramp to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ramps represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Ramp3DReadOnly other, double epsilon)
   {
      return getSize().epsilonEquals(other.getSize(), epsilon) && getPosition().geometricallyEquals(other.getPosition(), epsilon)
            && getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
   }

   /**
    * Changes the given {@code transformable} from being expressed in world to being expressed in this
    * shape local coordinates.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *           Modified.
    */
   default void transformToLocal(Transformable transformable)
   {
      transformable.applyInverseTransform(getPose());
   }

   /**
    * Changes the given {@code transformable} from being expressed in this shape local coordinates to
    * being expressed in world.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *           Modified.
    */
   default void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(getPose());
   }
}
