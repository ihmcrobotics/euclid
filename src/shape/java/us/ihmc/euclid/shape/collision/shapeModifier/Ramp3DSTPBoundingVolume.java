package us.ihmc.euclid.shape.collision.shapeModifier;

import java.util.function.BiConsumer;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Ramp3DSTPBoundingVolume extends SphereTorusPatchesBoundingVolume<Ramp3DReadOnly>
{
   private final UnitVector3D supportDirectionLocal = new UnitVector3D();
   private final Point3D faceSphereCenter = new Point3D();
   private final Point3D closestVertexCenter = new Point3D();

   private final Vector3D limitPlaneNormal = new Vector3D();
   private final Vector3D toFaceCentroid = new Vector3D();

   private final Point3D closestFaceEdgeCenter0 = new Point3D();
   private final Point3D closestFaceEdgeCenter1 = new Point3D();
   private final Point3D closestFaceEdgeCenter2 = new Point3D();
   private final Point3D[] closestFaceEdgeCenters = {closestFaceEdgeCenter0, closestFaceEdgeCenter1, closestFaceEdgeCenter2};
   private final Vector3D closestFaceEdgeAxis0 = new Vector3D();
   private final Vector3D closestFaceEdgeAxis1 = new Vector3D();
   private final Vector3D closestFaceEdgeAxis2 = new Vector3D();
   private final Vector3D[] closestFaceEdgeAxes = {closestFaceEdgeAxis0, closestFaceEdgeAxis1, closestFaceEdgeAxis2};
   private final double[] closestFaceEdgeLengths = {0, 0, 0};

   public Ramp3DSTPBoundingVolume()
   {
   }

   @Override
   protected double findMaximumEdgeLengthSquared()
   {
      double rampLength = shape3D.getRampLength();
      Vector3DReadOnly size = shape3D.getSize();
      return EuclidCoreTools.max(EuclidCoreTools.normSquared(size.getX(), size.getY()),
                                 EuclidCoreTools.normSquared(size.getY(), size.getZ()),
                                 EuclidCoreTools.normSquared(rampLength, size.getY()));
   }

   private enum Face
   {
      RAMP((shape3D, centroid) -> centroid.set(0.5 * shape3D.getSizeX(), 0.0, 0.5 * shape3D.getSizeZ())),
      X_MAX((shape3D, centroid) -> centroid.set(shape3D.getSizeX(), 0.0, 0.5 * shape3D.getSizeZ())),
      Y_MIN((shape3D, centroid) -> centroid.set(shape3D.getCentroid().getX(), -0.5 * shape3D.getSizeY(), shape3D.getCentroid().getZ())),
      Y_MAX((shape3D, centroid) -> centroid.set(shape3D.getCentroid().getX(), 0.5 * shape3D.getSizeY(), shape3D.getCentroid().getZ())),
      Z_MIN((shape3D, centroid) -> centroid.set(0.5 * shape3D.getSizeX(), 0.0, 0.0));

      private final BiConsumer<Ramp3DReadOnly, Tuple3DBasics> faceCentroidCalculator;

      private Face(BiConsumer<Ramp3DReadOnly, Tuple3DBasics> faceCentroidCalculator)
      {
         this.faceCentroidCalculator = faceCentroidCalculator;
      }

      public void getFaceCentroid(Ramp3DReadOnly shape3D, Tuple3DBasics faceCentroid)
      {
         faceCentroidCalculator.accept(shape3D, faceCentroid);
      }
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (shape3D.getPose().hasRotation())
         shape3D.getPose().inverseTransform(supportDirection, supportDirectionLocal);
      else
         supportDirectionLocal.set(supportDirection);
      supportDirection = supportDirectionLocal;

      Vector3DReadOnly rampSurfaceNormal = shape3D.getRampSurfaceNormal();
      Vector3DReadOnly size = shape3D.getSize();

      double rampDot = supportDirection.dot(rampSurfaceNormal);
      double faceXPlusDot = supportDirection.getX();
      double faceYPlusDot = supportDirection.getY();
      double faceZPlusDot = supportDirection.getZ();
      double faceYAbsDot = Math.abs(faceYPlusDot);
      boolean isFaceYMaxCloser = faceYPlusDot > 0.0;

      Face firstClosestFace;

      faceSphereCenter.setToZero();

      if (EuclidShapeTools.isFirstValueMaximum(faceXPlusDot, faceYAbsDot, -faceZPlusDot, rampDot))
      {
         firstClosestFace = Face.X_MAX;
         faceSphereCenter.setX(size.getX() - sphereOffset(firstClosestFace));
         faceSphereCenter.setZ(0.5 * size.getZ());

         closestFaceEdgeLengths[0] = size.getZ();
         closestFaceEdgeLengths[1] = size.getY();
         closestFaceEdgeLengths[2] = size.getY();
         closestFaceEdgeAxis0.set(Axis3D.Z);
         closestFaceEdgeAxis1.set(Axis3D.Y);
         closestFaceEdgeAxis2.set(Axis3D.Y);
         closestFaceEdgeCenter0.set(size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
         closestFaceEdgeCenter1.set(size.getX(), 0.0, 0.0);
         closestFaceEdgeCenter2.set(size.getX(), 0.0, size.getZ());
      }
      else if (EuclidShapeTools.isFirstValueMaximum(faceYAbsDot, -faceZPlusDot, rampDot))
      {
         firstClosestFace = isFaceYMaxCloser ? Face.Y_MAX : Face.Y_MIN;
         faceSphereCenter.setX(0.5 * size.getX());
         faceSphereCenter.setY(isFaceYMaxCloser ? 0.5 * size.getY() - sphereOffset(firstClosestFace) : -0.5 * size.getY() + sphereOffset(firstClosestFace));
         faceSphereCenter.setZ(0.5 * size.getZ());

         closestFaceEdgeLengths[0] = size.getZ();
         closestFaceEdgeLengths[1] = size.getX();
         closestFaceEdgeLengths[2] = shape3D.getRampLength();
         closestFaceEdgeAxis0.set(Axis3D.Z);
         closestFaceEdgeAxis1.set(Axis3D.X);
         closestFaceEdgeAxis2.set(rampSurfaceNormal.getZ(), 0.0, -rampSurfaceNormal.getX());
         closestFaceEdgeCenter0.set(size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
         closestFaceEdgeCenter1.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.0);
         closestFaceEdgeCenter2.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
      }
      else if (-faceZPlusDot > rampDot)
      {
         firstClosestFace = Face.Z_MIN;
         faceSphereCenter.setX(0.5 * size.getX());
         faceSphereCenter.setZ(sphereOffset(firstClosestFace));

         closestFaceEdgeLengths[0] = size.getY();
         closestFaceEdgeLengths[1] = size.getX();
         closestFaceEdgeLengths[2] = size.getY();
         closestFaceEdgeAxis0.set(Axis3D.Y);
         closestFaceEdgeAxis1.set(Axis3D.X);
         closestFaceEdgeAxis2.set(Axis3D.Y);
         closestFaceEdgeCenter0.set(size.getX(), 0.0, 0.0);
         closestFaceEdgeCenter1.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.0);
         closestFaceEdgeCenter2.set(0.0, 0.0, 0.0);
      }
      else
      {
         firstClosestFace = Face.RAMP;
         faceSphereCenter.setX(0.5 * size.getX());
         faceSphereCenter.setZ(0.5 * size.getZ());
         faceSphereCenter.scaleAdd(-sphereOffset(firstClosestFace), rampSurfaceNormal, faceSphereCenter);

         closestFaceEdgeLengths[0] = size.getY();
         closestFaceEdgeLengths[1] = shape3D.getRampLength();
         closestFaceEdgeLengths[2] = size.getY();
         closestFaceEdgeAxis0.set(Axis3D.Y);
         closestFaceEdgeAxis1.set(rampSurfaceNormal.getZ(), 0.0, -rampSurfaceNormal.getX());
         closestFaceEdgeAxis2.set(Axis3D.Y);
         closestFaceEdgeCenter0.set(size.getX(), 0.0, size.getZ());
         closestFaceEdgeCenter1.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
         closestFaceEdgeCenter2.set(0.0, 0.0, 0.0);
      }

      double limitScaleFactor = 0.5 * largeRadius / (largeRadius - smallRadius);
      firstClosestFace.getFaceCentroid(shape3D, toFaceCentroid);
      toFaceCentroid.sub(faceSphereCenter);

      int isOutsideCounter = 0;
      boolean isWithinFace = true;
      boolean hasComputedSupportingVertex = false;

      for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
      {
         /*
          * For each edge we construct a plane which the edge and the faceSphereCenter belong to, then we
          * test for whether the face centroid and support direction are on the same side of the plane. If
          * not the supportDirection cannot be handled by the face. If the support direction violates 2+
          * edges and the support direction is not handled by either edge, then the support direction can
          * only be handled by a ramp's vertex.
          */
         Point3D edgeCenter = closestFaceEdgeCenters[edgeIndex];
         Vector3D edgeAxis = closestFaceEdgeAxes[edgeIndex];
         double edgeLength = closestFaceEdgeLengths[edgeIndex];

         limitPlaneNormal.sub(edgeCenter, faceSphereCenter);
         limitPlaneNormal.cross(edgeAxis);

         if (toFaceCentroid.dot(limitPlaneNormal) * supportDirection.dot(limitPlaneNormal) < 0.0)
         {
            /*
             * The face cannot handle the supportVertex, then maybe the edge is. Here we simply evaluate the
             * supportingVertex and test that it is within the limits.
             */
            isOutsideCounter++;
            isWithinFace = false;

            double torusRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, edgeLength);
            EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, edgeCenter, edgeAxis, torusRadius, largeRadius, supportingVertexToPack);

            if (Math.abs(EuclidGeometryTools.percentageAlongLine3D(supportingVertexToPack, edgeCenter, edgeAxis)) <= edgeLength * limitScaleFactor)
            {
               hasComputedSupportingVertex = true;
               break;
            }

            if (isOutsideCounter >= 2)
            {
               EuclidShapeTools.supportingVectexRamp3D(supportDirection, size, closestVertexCenter);
               EuclidShapeTools.supportingVertexSphere3D(supportDirection, closestVertexCenter, smallRadius, supportingVertexToPack);
               hasComputedSupportingVertex = true;
               break;
            }
         }
      }

      if (!hasComputedSupportingVertex)
      {
         if (isWithinFace)
         {
            EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);
         }
         else
         {
            EuclidShapeTools.supportingVectexRamp3D(supportDirection, size, closestVertexCenter);
            EuclidShapeTools.supportingVertexSphere3D(supportDirection, closestVertexCenter, smallRadius, supportingVertexToPack);
         }
      }

      shape3D.transformToWorld(supportingVertexToPack);

      return true;
   }

   private double sphereOffset(Face face)
   {
      double a, b;
      double diagonalSquared;

      switch (face)
      {
         case RAMP:
            a = shape3D.getRampLength();
            b = shape3D.getSizeY();
            diagonalSquared = a * a + b * b;
            break;
         case X_MAX:
            a = shape3D.getSizeY();
            b = shape3D.getSizeZ();
            diagonalSquared = a * a + b * b;
            break;
         case Y_MAX:
         case Y_MIN:
            diagonalSquared = shape3D.getRampLength() * shape3D.getRampLength();
            break;
         case Z_MIN:
            a = shape3D.getSizeX();
            b = shape3D.getSizeY();
            diagonalSquared = a * a + b * b;
            break;
         default:
            throw new IllegalStateException();
      }
      double radius = largeRadius - smallRadius;
      return Math.sqrt(radius * radius - 0.25 * diagonalSquared);
   }
}
