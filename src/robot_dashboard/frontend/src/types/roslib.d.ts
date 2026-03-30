declare module "roslib" {
  class Ros {
    constructor(options?: Record<string, any>);
    connect(url: string): void;
    close(): void;
    on(event: string, callback: (...args: any[]) => void): void;
    isConnected: boolean;
    getTopics(
      callback: (result: { topics: string[]; types: string[] }) => void,
      failedCallback?: (error: string) => void
    ): void;
    getServices(
      callback: (serviceNames: string[]) => void,
      failedCallback?: (error: string) => void
    ): void;
    getServiceType(
      service: string,
      callback: (type: string) => void,
      failedCallback?: (error: string) => void
    ): void;
  }

  class ActionClient {
    constructor(options: {
      ros: Ros;
      serverName: string;
      actionName: string;
    });
    dispose(): void;
  }

  class Goal {
    constructor(options: {
      actionClient: ActionClient;
      goalMessage: any;
    });
    on(event: string, callback: (...args: any[]) => void): void;
    send(timeout?: number): void;
    cancel(): void;
  }

  class Topic {
    constructor(options: {
      ros: Ros;
      name: string;
      messageType: string;
      throttle_rate?: number;
      queue_size?: number;
      latch?: boolean;
    });
    subscribe(callback: (message: any) => void): void;
    unsubscribe(callback?: (message: any) => void): void;
    publish(message: any): void;
  }

  class Service {
    constructor(options: {
      ros: Ros;
      name: string;
      serviceType: string;
    });
    callService(
      request: any,
      callback: (response: any) => void,
      failedCallback?: (error: string) => void
    ): void;
  }

  class ServiceRequest {
    constructor(values?: Record<string, any>);
  }

  class Message {
    constructor(values?: Record<string, any>);
  }
}
