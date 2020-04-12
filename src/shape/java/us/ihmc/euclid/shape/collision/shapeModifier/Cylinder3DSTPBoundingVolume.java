package us.ihmc.euclid.shape.collision.shapeModifier;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Cylinder3DSTPBoundingVolume extends SphereTorusPatchesBoundingVolume<Cylinder3DReadOnly>
{
   public Cylinder3DSTPBoundingVolume()
   {
   }

   @Override
   protected double findMaximumEdgeLengthSquared()
   {
      double maximumEdgeLengthSquared = Math.max(shape3D.getLength(), 2.0 * shape3D.getRadius());
      return maximumEdgeLengthSquared * maximumEdgeLengthSquared;
   }

   private final Vector3D orthogonalToAxis = new Vector3D();
   private final Point3D sideSphereCenter = new Point3D();
   private final Point3D edgeSphereCenter = new Point3D();
   private final Point3D capSphereCenter = new Point3D();

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      double cylinderRadius = shape3D.getRadius();
      double cylinderHalfLength = shape3D.getHalfLength();
      UnitVector3DReadOnly cylinderAxis = shape3D.getAxis();
      Point3DReadOnly cylinderPosition = shape3D.getPosition();

      orthogonalToAxis.set(supportDirection);

      double dot = supportDirection.dot(cylinderAxis);
      double sign = dot > 0.0 ? 1.0 : -1.0;
      orthogonalToAxis.setAndScale(dot, cylinderAxis);
      orthogonalToAxis.sub(supportDirection, orthogonalToAxis);

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
         sideSphereCenter.setAndScale(cylinderRadius - sideSphereRadius, orthogonalToAxis);

         edgeSphereCenter.setAndScale(cylinderRadius, orthogonalToAxis);
         edgeSphereCenter.scaleAdd(sign * cylinderHalfLength, cylinderAxis, edgeSphereCenter);
      }

      double capSphereRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, 2.0 * cylinderRadius);
      capSphereCenter.setAndScale(sign * (cylinderHalfLength - capSphereRadius), cylinderAxis);

      if (!getSideSupportingVertex(supportDirection, supportingVertexToPack))
      {
         if (!getCapSupportingVertex(supportDirection, supportingVertexToPack))
         {
            EuclidShapeTools.supportingVertexSphere3D(supportDirection, edgeSphereCenter, smallRadius, supportingVertexToPack);
         }
      }

      supportingVertexToPack.add(cylinderPosition);

      return true;
   }

   private boolean getSideSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (sideSphereCenter.containsNaN())
         return false;
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, sideSphereCenter, largeRadius, supportingVertexToPack);
      double dot = TupleTools.dot(supportingVertexToPack, shape3D.getAxis());
      return dot <= shape3D.getHalfLength() && dot >= -shape3D.getHalfLength();
   }

   private final Point3D validationPoint = new Point3D();

   private boolean getCapSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, capSphereCenter, largeRadius, supportingVertexToPack);
      double dot = TupleTools.dot(supportingVertexToPack, shape3D.getAxis());
      validationPoint.setAndScale(dot, shape3D.getAxis());
      validationPoint.sub(supportingVertexToPack, validationPoint);
      return validationPoint.distanceFromOriginSquared() <= shape3D.getRadius() * shape3D.getRadius();
   }
}
