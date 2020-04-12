package us.ihmc.euclid.shape.collision.shapeModifier;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Capsule3DSTPBoundingVolume extends SphereTorusPatchesBoundingVolume<Capsule3DReadOnly>
{
   public Capsule3DSTPBoundingVolume()
   {
   }

   @Override
   protected double findMaximumEdgeLengthSquared()
   {
      return shape3D.getLength() * shape3D.getLength();
   }

   @Override
   protected void updateRadii()
   {
      super.updateRadii();

      smallRadius += shape3D.getRadius();
      largeRadius += shape3D.getRadius();
   }

   private final Vector3D orthogonalToAxis = new Vector3D();
   private final Point3D sideSphereCenter = new Point3D();
   private final Point3D edgeSphereCenter = new Point3D();

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      double capsuleHalfLength = shape3D.getHalfLength();
      UnitVector3DReadOnly capsuleAxis = shape3D.getAxis();
      Point3DReadOnly capsulePosition = shape3D.getPosition();

      orthogonalToAxis.set(supportDirection);

      double dot = supportDirection.dot(capsuleAxis);
      double sign = dot > 0.0 ? 1.0 : -1.0;
      orthogonalToAxis.setAndScale(dot, capsuleAxis);
      orthogonalToAxis.sub(supportDirection, orthogonalToAxis);
      edgeSphereCenter.setAndScale(sign * capsuleHalfLength, capsuleAxis);

      double distanceSquaredFromAxis = orthogonalToAxis.lengthSquared();

      if (distanceSquaredFromAxis < EuclidShapeTools.MIN_DISTANCE_EPSILON)
      {
         sideSphereCenter.setToNaN();
         edgeSphereCenter.setToNaN();
      }
      else
      {
         orthogonalToAxis.scale(1.0 / EuclidCoreTools.squareRoot(distanceSquaredFromAxis));

         double sideSphereRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, shape3D.getLength());
         sideSphereCenter.setAndScale(-sideSphereRadius, orthogonalToAxis);
      }

      if (!getSideSupportingVertex(supportDirection, supportingVertexToPack))
      {
         EuclidShapeTools.supportingVertexSphere3D(supportDirection, edgeSphereCenter, smallRadius, supportingVertexToPack);
      }

      supportingVertexToPack.add(capsulePosition);

      return true;
   }

   private boolean getSideSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (sideSphereCenter.containsNaN())
         return false;
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, sideSphereCenter, largeRadius, supportingVertexToPack);
      double dotMax = shape3D.getHalfLength() * largeRadius / (largeRadius - smallRadius);
      double dot = TupleTools.dot(supportingVertexToPack, shape3D.getAxis());
      return Math.abs(dot) <= Math.abs(dotMax);
   }
}
