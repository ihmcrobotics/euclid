package us.ihmc.euclid.shape.interfaces;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface IntermediateVariableSupplier
{
   Point3DBasics getPoint3D(int index);
   Vector3DBasics getVector3D(int index);

   public static IntermediateVariableSupplier defaultIntermediateVariableSupplier()
   {
      return new IntermediateVariableSupplier()
      {
         @Override
         public Point3DBasics getPoint3D(int index)
         {
            return new Point3D();
         }

         @Override
         public Vector3DBasics getVector3D(int index)
         {
            return new Vector3D();
         }
      };
   }

   public static IntermediateVariableSupplier garbageFreeIntermediateVariableSupplier()
   {
      return new IntermediateVariableSupplier()
      {
         private final List<Point3D> point3DPool = new ArrayList<>();
         private final List<Vector3D> vector3DPool = new ArrayList<>();

         @Override
         public Point3DBasics getPoint3D(int index)
         {
            while (point3DPool.size() <= index)
               point3DPool.add(new Point3D());
            return point3DPool.get(index);
         }

         @Override
         public Vector3DBasics getVector3D(int index)
         {
            while (vector3DPool.size() <= index)
               vector3DPool.add(new Vector3D());
            return vector3DPool.get(index);
         }
      };
   }
}
