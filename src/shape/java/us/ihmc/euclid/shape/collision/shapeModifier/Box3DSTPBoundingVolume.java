package us.ihmc.euclid.shape.collision.shapeModifier;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.isPoint3DAbovePlane3D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.triangleIsoscelesHeight;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Box3DSTPBoundingVolume extends SphereTorusPatchesBoundingVolume<Box3DReadOnly>
{
   private final Vector3D halfSize = new Vector3D();
   private final UnitVector3D supportUnitDirection = new UnitVector3D();
   private final Point3D faceSphereCenter = new Point3D();
   private final Point3D closestVertexCenter = new Point3D();

   private final Vector3D limitPlaneNormal = new Vector3D();
   private final Point3D faceCenter = new Point3D();
   private final Point3D edgeCenter = new Point3D();
   private final Point3D queryAbsolute = new Point3D();

   public Box3DSTPBoundingVolume()
   {
   }

   @Override
   protected double findMaximumEdgeLengthSquared()
   {
      Vector3DReadOnly size = shape3D.getSize();
      return EuclidCoreTools.max(EuclidCoreTools.normSquared(size.getX(), size.getY()),
                                 EuclidCoreTools.normSquared(size.getX(), size.getZ()),
                                 EuclidCoreTools.normSquared(size.getY(), size.getZ()));
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (shape3D.getPose().hasRotation())
      {
         shape3D.getPose().inverseTransform(supportDirection, supportUnitDirection);
      }
      else
         supportUnitDirection.set(supportDirection);
      supportDirection = supportUnitDirection;

      halfSize.setAndScale(0.5, shape3D.getSize());

      double faceXPlusDot = supportDirection.getX();
      double faceYPlusDot = supportDirection.getY();
      double faceZPlusDot = supportDirection.getZ();

      double faceXAbsDot = Math.abs(faceXPlusDot);
      double faceYAbsDot = Math.abs(faceYPlusDot);
      double faceZAbsDot = Math.abs(faceZPlusDot);

      boolean isFaceXMaxCloser = faceXPlusDot > 0.0;
      boolean isFaceYMaxCloser = faceYPlusDot > 0.0;
      boolean isFaceZMaxCloser = faceZPlusDot > 0.0;

      Axis3D firstClosestFace;
      boolean isMaxFace;

      if (faceXAbsDot > faceYAbsDot)
      {
         if (faceXAbsDot > faceZAbsDot)
         { // Closest is one of the 2 x-faces
            firstClosestFace = Axis3D.X;
            isMaxFace = isFaceXMaxCloser;
         }
         else
         { // Closest is one of the 2 z-faces
            firstClosestFace = Axis3D.Z;
            isMaxFace = isFaceZMaxCloser;
         }
      }
      else if (faceYAbsDot > faceZAbsDot)
      { // Closest is one of the 2 y-faces
         firstClosestFace = Axis3D.Y;
         isMaxFace = isFaceYMaxCloser;
      }
      else
      { // Closest is one of the 2 z-faces
         firstClosestFace = Axis3D.Z;
         isMaxFace = isFaceZMaxCloser;
      }

      double faceSphereOffset = getFaceSphereOffset(firstClosestFace);
      faceCenter.setToZero();
      faceSphereCenter.setToZero();
      faceCenter.setElement(firstClosestFace, halfSize.getElement(firstClosestFace));
      faceSphereCenter.setElement(firstClosestFace, halfSize.getElement(firstClosestFace) - faceSphereOffset);

      if (!isMaxFace)
      {
         faceCenter.negate();
         faceSphereCenter.negate();
      }

      EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);

      if (!isFaceSphereSupportingVertexValid(firstClosestFace, faceSphereOffset, supportingVertexToPack))
      {
         closestVertexCenter.set(isFaceXMaxCloser ? halfSize.getX() : -halfSize.getX(),
                                 isFaceYMaxCloser ? halfSize.getY() : -halfSize.getY(),
                                 isFaceZMaxCloser ? halfSize.getZ() : -halfSize.getZ());

         EuclidShapeTools.supportingVertexSphere3D(supportDirection, closestVertexCenter, smallRadius, supportingVertexToPack);

         /*
          * Scale factor deduced from Thales similar triangle theorem and given that the edge's torus, which
          * tube radius is largeRadius, meets the vertex smallRadius.
          */
         double limitScaleFactor = largeRadius / (largeRadius - smallRadius);

         for (Axis3D axis : Axis3D.values)
         { // We look at whether the supportingVertex actually belongs to the torus of a neighboring edge.
            if (Math.abs(supportingVertexToPack.getElement(axis)) < halfSize.getElement(axis) * limitScaleFactor)
            {
               edgeCenter.setElement(axis, 0.0);
               edgeCenter.setElement(axis.next(), closestVertexCenter.getElement(axis.next()));
               edgeCenter.setElement(axis.previous(), closestVertexCenter.getElement(axis.previous()));

               double torusRadius = triangleIsoscelesHeight(largeRadius - smallRadius, shape3D.getSize().getElement(axis));
               EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, edgeCenter, axis, torusRadius, largeRadius, supportingVertexToPack);
               break;
            }
         }
      }

      shape3D.transformToWorld(supportingVertexToPack);

      return true;
   }

   private boolean isFaceSphereSupportingVertexValid(Axis3D face, double faceSphereOffset, Point3DReadOnly query)
   {
      /*
       * Testing that the query is inside the pyramid which apex is located at the face's large sphere
       * center and legs are extended to infinity while going through each face's vertices. The comparison
       * is done by testing that building one at a time each side plane of the pyramid and testing that
       * the face's center and the query are on the same side of the plane. In addition, the tests are
       * performed in the positive octant of the box only, this way, the number of tests is reduced to 2
       * instead of 4: only the 2 positive sides of the pyramid have to be tested for.
       */
      queryAbsolute.setAndAbsolute(query);

      faceCenter.setToZero();
      faceCenter.setElement(face, halfSize.getElement(face));

      // Testing against first side
      edgeCenter.set(faceCenter);
      edgeCenter.setElement(face.next(), halfSize.getElement(face.next()));
      Axis3D edgeAxis = face.previous();

      limitPlaneNormal.sub(edgeCenter, faceCenter);
      limitPlaneNormal.setElement(face, limitPlaneNormal.getElement(face) + faceSphereOffset);
      limitPlaneNormal.cross(edgeAxis);

      if (isPoint3DAbovePlane3D(faceCenter, edgeCenter, limitPlaneNormal) != isPoint3DAbovePlane3D(queryAbsolute, edgeCenter, limitPlaneNormal))
         return false;

      // Testing against second side
      edgeCenter.set(faceCenter);
      edgeCenter.setElement(face.previous(), halfSize.getElement(face.previous()));
      edgeAxis = face.next();

      limitPlaneNormal.sub(edgeCenter, faceCenter);
      limitPlaneNormal.setElement(face, limitPlaneNormal.getElement(face) + faceSphereOffset);
      limitPlaneNormal.cross(edgeAxis);

      if (isPoint3DAbovePlane3D(faceCenter, edgeCenter, limitPlaneNormal) != isPoint3DAbovePlane3D(queryAbsolute, edgeCenter, limitPlaneNormal))
         return false;

      return true;
   }

   private double getFaceSphereOffset(Axis3D face)
   {
      double a = halfSize.getElement(face.next());
      double b = halfSize.getElement(face.previous());
      return Math.sqrt(EuclidCoreTools.square(largeRadius - smallRadius) - (a * a + b * b));
   }
}
