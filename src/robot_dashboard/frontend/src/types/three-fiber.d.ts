declare module "@react-three/fiber" {
  export const Canvas: any;
  export function useFrame(callback: (state: any, delta: number) => void): void;
  export function useThree(): any;
}

declare module "@react-three/drei" {
  export const OrbitControls: any;
  export const Grid: any;
  export const Line: any;
  export const Text: any;
  export const Html: any;
}
