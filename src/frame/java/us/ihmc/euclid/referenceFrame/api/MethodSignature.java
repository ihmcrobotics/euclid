package us.ihmc.euclid.referenceFrame.api;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

public class MethodSignature
{
   private String name;
   private int modifiers;
   private List<Class<?>> parameterTypes;
   private Class<?> returnType;

   public MethodSignature()
   {
   }

   public MethodSignature(String name, Class<?>... parameterTypes)
   {
      setName(name);
      setParameterTypes(parameterTypes);
   }

   public MethodSignature(MethodSignature other)
   {
      set(other);
   }

   public MethodSignature(Method method)
   {
      set(method);
   }

   public void set(MethodSignature other)
   {
      name = other.getName();
      modifiers = other.getModifiers();
      setParameterTypes(other.toParameterTypeArray());
      returnType = other.getReturnType();
   }

   public void set(Method method)
   {
      name = method.getName();
      modifiers = method.getModifiers();
      setParameterTypes(method.getParameterTypes());
      returnType = method.getReturnType();
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setModifiers(int modifiers)
   {
      this.modifiers = modifiers;
   }

   public int getParameterCount()
   {
      return parameterTypes.size();
   }

   public void setParameterTypes(Class<?>[] parameterTypes)
   {
      this.parameterTypes = new ArrayList<>(Arrays.asList(parameterTypes));
   }

   public void addParameterType(int index, Class<?> parameterType)
   {
      parameterTypes.add(index, parameterType);
   }

   public void removeParameterType(int index)
   {
      parameterTypes.remove(index);
   }

   public void setParameterType(int index, Class<?> parameterType)
   {
      parameterTypes.set(index, parameterType);
   }

   public void setReturnType(Class<?> returnType)
   {
      this.returnType = returnType;
   }

   public String getName()
   {
      return name;
   }

   public int getModifiers()
   {
      return modifiers;
   }

   public boolean isStatic()
   {
      return Modifier.isStatic(modifiers);
   }

   public boolean isPublic()
   {
      return Modifier.isStatic(modifiers);
   }

   public Class<?>[] toParameterTypeArray()
   {
      return parameterTypes.toArray(new Class<?>[0]);
   }

   public List<Class<?>> getParameterTypes()
   {
      return parameterTypes;
   }

   public Class<?> getParameterType(int index)
   {
      return parameterTypes.get(index);
   }

   public Class<?> getReturnType()
   {
      return returnType;
   }

   public String getMethodSimpleName()
   {
      return getMethodSimpleName(returnType, name, parameterTypes);
   }

   public static String getMethodSimpleName(Method method)
   {
      return getMethodSimpleName(method.getReturnType(), method.getName(), method.getParameterTypes());
   }

   public static String getMethodSimpleName(Class<?> returnType, String methodName, Class<?>... parameterTypes)
   {
      return getMethodSimpleName(returnType, methodName, Arrays.asList(parameterTypes));
   }

   public static String getMethodSimpleName(Class<?> returnType, String methodName, Collection<Class<?>> parameterTypes)
   {
      String returnTypeName = returnType == null ? "void" : returnType.getSimpleName();
      return returnTypeName + " " + methodName + EuclidCoreIOTools.getCollectionString("(", ")", ", ", parameterTypes, Class::getSimpleName);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MethodSignature)
      {
         MethodSignature other = (MethodSignature) object;

         if (name == null ? other.name != null : !name.equals(other.name))
            return false;
         if (modifiers != other.modifiers)
            return false;
         if (parameterTypes == null ? other.parameterTypes != null : !parameterTypes.equals(other.parameterTypes))
            return false;
         if (returnType != other.returnType)
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
